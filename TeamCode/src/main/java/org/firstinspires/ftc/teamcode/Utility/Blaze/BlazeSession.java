package org.firstinspires.ftc.teamcode.Utility.Blaze;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.BlazeConstants;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicLongArray;

import dev.anygeneric.blazeftc.AcceleratedMotor;
import dev.anygeneric.blazeftc.BlazeDummyPlug;
import dev.anygeneric.blazeftc.BlazeDummyPlugKt;
import dev.anygeneric.blazeftc.BlazeFTC;
import dev.anygeneric.blazeftc.Hub;
import dev.anygeneric.blazeftc.InterfaceAccessor;
import kotlin.Unit;

public final class BlazeSession {

    public static final int CONTROL_HUB = 0, EXPANSION_HUB = 1;
    public static final BlazeRing[] BULK = {new BlazeRing(), new BlazeRing()};
    public static final BlazeRing PINPOINT = new BlazeRing();
    private static final LynxModule[] FED = new LynxModule[2];
    private static final AtomicLongArray LAST_BULK_NS = new AtomicLongArray(2);
    //frozen data drove the turret round
    private static final long BULK_STALE_NS = 50_000_000L;
    private static List<LynxModule> hubs = Collections.emptyList();
    private static HardwareMap map;
    private static boolean open, running, nextOwned;
    private static BlazePinpointLocalizer pinpoint;
    public static String fault = "";

    private BlazeSession() {}

    public static boolean begin(HardwareMap hardwareMap) {
        return begin(hardwareMap, false);
    }

    public static boolean beginNext(HardwareMap hardwareMap) {
        return begin(hardwareMap, true);
    }

    private static synchronized boolean begin(HardwareMap hardwareMap, boolean next) {

        end();
        fault = "";
        nextOwned = next;
        hubs = hardwareMap.getAll(LynxModule.class);
        map = hardwareMap;
        if (!BlazeConstants.ENABLED) return false;
        try {
            BlazeDummyPlug.initializeBlazeFTC(hardwareMap);
            open = true;
            if (BlazeConstants.MOTOR_ACCELERATION) BlazeDummyPlug.engageMotorAccel(hardwareMap);
            return true;
        } catch (Throwable t) {
            fail("start", t);
            return open;
        }
    }

    public static synchronized void engage(HardwareMap hardwareMap, Follower follower) {

        if (!open) return;
        LynxModule control = find(true), expansion = find(false);
        if (BlazeConstants.BULK_CONTROL_HUB && control != null) bulk(hardwareMap, Hub.CtrlHub, CONTROL_HUB, control, Math.max(1, Math.min(2, BlazeConstants.CONTROL_HUB_PACKETS)));
        if (BlazeConstants.BULK_EXPANSION_HUB && expansion != null) bulk(hardwareMap, Hub.ExHub, EXPANSION_HUB, expansion, 1);
        attachPinpoint(follower);
    }

    public static synchronized void attachPinpoint(Follower follower) {

        if (!open || follower == null || !BlazeConstants.PINPOINT_ACCELERATION) return;
        try {
            pinpoint = BlazePinpointLocalizer.attach(follower, PINPOINT);
        } catch (Throwable t) {
            fail("Pinpoint", t);
        }
    }

    private static void bulk(HardwareMap hardwareMap, Hub hub, final int index, LynxModule module, int packets) {

        try {
            final BlazeRing ring = BULK[index];
            BlazeDummyPlug.engageBulkReadAcceleration(hardwareMap, hub, packets, () -> {
                long now = System.nanoTime();
                LAST_BULK_NS.set(index, now);
                ring.push(now, ring.total, 0, 0, 0, 0, 0);
                return Unit.INSTANCE;
            });
            FED[index] = module;
        } catch (Throwable t) {
            fail(hub + " bulk reads", t);
        }
    }

    static synchronized void markFed(int index, LynxModule module) {
        FED[index] = module;
    }

    public static LynxModule find(boolean control) {

        for (LynxModule m : hubs) {
            InterfaceAccessor.ModuleStatus s;
            try {
                s = BlazeDummyPlugKt.module_status(m);
            } catch (Throwable t) {
                continue;
            }
            if (control && s == InterfaceAccessor.ModuleStatus.Internal) return m;
            if (!control && (s == InterfaceAccessor.ModuleStatus.RS485 || s == InterfaceAccessor.ModuleStatus.USB)) return m;
        }
        return null;
    }

    public static void tick() {

        if (open && !running) {
            running = true;
            if (!nextOwned) {
                try {
                    BlazeFTC.run(0);
                } catch (Throwable t) {
                    fail("run", t);
                }
            }
        }
        clearUnfed();
    }

    public static void clearUnfed() {

        for (int k = 0; k < hubs.size(); k++) {
            LynxModule m = hubs.get(k);
            if (!feeds(m)) m.clearBulkCache();
        }
    }

    public static boolean feeds(LynxModule module) {

        if (module == null || !running) return false;
        for (int i = 0; i < 2; i++) {
            if (module == FED[i]) return System.nanoTime() - LAST_BULK_NS.get(i) < BULK_STALE_NS;
        }
        return false;
    }

    public static synchronized void end() {

        if (open && !nextOwned) {
            try {
                BlazeDummyPlug.closeBlazeFTC();
            } catch (Throwable t) {
                RobotLog.ee("Blaze", t, "close failed");
            }
        }
        if (open) {
            for (LynxModule m : hubs) {
                try {
                    m.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                } catch (Throwable ignored) {
                }
            }
        }
        restoreMotors();
        open = running = nextOwned = false;
        map = null;
        FED[0] = FED[1] = null;
        LAST_BULK_NS.set(0, 0);
        LAST_BULK_NS.set(1, 0);
        pinpoint = null;
        BULK[0].clear();
        BULK[1].clear();
        PINPOINT.clear();
    }

    //the map outlives the OpMode, give the SDK motors back
    private static void restoreMotors() {

        if (map == null) return;
        try {
            for (String name : map.getAllNames(DcMotorEx.class)) {
                DcMotorEx m = map.get(DcMotorEx.class, name);
                if (!(m instanceof AcceleratedMotor)) continue;
                map.dcMotor.remove(name);
                map.dcMotor.put(name, ((AcceleratedMotor) m).getDcMotorEx());
            }
        } catch (Throwable t) {
            RobotLog.ee("Blaze", t, "couldn't put the SDK motors back");
        }
    }

    //ms since that hub's last Blaze bulk update, NaN when Blaze doesn't feed it
    public static double bulkAgeMs(int index) {

        long t = LAST_BULK_NS.get(index);
        if (FED[index] == null || !running || t == 0) return Double.NaN;
        return (System.nanoTime() - t) * 1e-6;
    }

    public static boolean fedNow(int index) {
        return FED[index] != null && feeds(FED[index]);
    }

    public static boolean isOpen() {
        return open;
    }

    public static boolean isRunning() {
        return running;
    }

    public static BlazePinpointLocalizer pinpoint() {
        return pinpoint;
    }

    public static String status() {

        if (!BlazeConstants.ENABLED) return "off (BlazeConstants.ENABLED)";
        if (!open) return "not started" + (fault.isEmpty() ? "" : ": " + fault);
        String s = (running ? "running" : "ready") + ", bulk " + hubStatus("CH", 0) + hubStatus("EH", 1) + (pinpoint != null ? ", Pinpoint " + (pinpoint.usingSdk ? "SDK" : "Blaze") : "");
        return fault.isEmpty() ? s : s + " | " + fault;
    }

    private static String hubStatus(String name, int i) {

        if (FED[i] == null) return "";
        return name + (feeds(FED[i]) ? " live " : " STALE (normal reads) ");
    }

    private static void fail(String what, Throwable t) {

        fault = fault.isEmpty() ? what + ": " + t : fault + "; " + what + ": " + t;
        RobotLog.ee("Blaze", t, what + " failed, carrying on without it");
    }
}
