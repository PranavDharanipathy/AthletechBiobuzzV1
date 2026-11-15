package org.firstinspires.ftc.teamcode.Utility.Blaze;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Constants.BlazeConstants;

import java.lang.reflect.Field;

import dev.anygeneric.blazeftc.BlazeDummyPlug;
import dev.anygeneric.blazeftc.BlazeFTC;
import dev.anygeneric.blazeftc.Hub;
import dev.anygeneric.blazeftc_pedro.Pedro3SingleDataLocalizer;
import kotlin.Unit;

//Blaze as its README says (github.com/owenpmckenna/blaze_ftc): it reads the Pinpoint and the Expansion Hub itself and calls us
//the moment new data is in. The drive runs on each Pinpoint sample, the shooter's motors on each Expansion Hub bulk read
public final class BlazeSession {

    //Blaze's callbacks and the loop take turns on the robot's objects with this
    public static final Object LOCK = new Object();
    public static volatile long driveUpdates, mechanismUpdates;
    public static String fault = "";
    private static boolean open, running, pinpoint, expansionHub;

    private BlazeSession() {}

    //before any hardware is got
    public static synchronized boolean begin(HardwareMap hardwareMap) {

        end();
        fault = "";
        driveUpdates = mechanismUpdates = 0;
        if (!BlazeConstants.ENABLED) return false;
        try {
            BlazeDummyPlug.initializeBlazeFTC(hardwareMap);
            open = true;
            if (BlazeConstants.MOTOR_ACCELERATION) BlazeDummyPlug.engageMotorAccel(hardwareMap);
        } catch (Throwable t) {
            fail("start", t);
        }
        return open;
    }

    //from here Blaze owns follower.update(): drive must call it, nothing else may
    public static synchronized void drive(final Follower follower, final Runnable drive) {

        if (!open) return;
        try {
            Pedro3SingleDataLocalizer.setup(follower, () -> {

                synchronized (LOCK) {
                    drive.run();
                }
                driveUpdates++;
            });
            //its localizer can't setPose (relocalizing at the GARDEN); final field, so the same reflection Blaze uses to put its own in
            Field localizer = Follower.class.getField("localizer");
            localizer.setAccessible(true);
            localizer.set(follower, new BlazeLocalizer(follower.localizer));
            pinpoint = true;
        } catch (Throwable t) {
            fail("Pinpoint", t);
        }
    }

    public static synchronized void mechanisms(HardwareMap hardwareMap, final Runnable mechanisms) {

        if (!open) return;
        try {
            BlazeDummyPlug.engageBulkReadAcceleration(hardwareMap, Hub.ExHub, BlazeConstants.EXPANSION_HUB_PACKETS, () -> {

                synchronized (LOCK) {
                    mechanisms.run();
                }
                mechanismUpdates++;
                return Unit.INSTANCE;
            });
            expansionHub = true;
        } catch (Throwable t) {
            fail("Expansion Hub bulk reads", t);
        }
    }

    //at START
    public static synchronized void run() {

        if (!open || running) return;
        try {
            BlazeFTC.run(0);
            running = true;
        } catch (Throwable t) {
            fail("run", t);
        }
    }

    //must happen or the robot restarts; MonitorOpModeHook also calls it after every OpMode
    public static synchronized void end() {

        if (open) {
            try {
                BlazeDummyPlug.closeBlazeFTC();
            } catch (Throwable t) {
                RobotLog.ee("Blaze", t, "close failed");
            }
        }
        open = running = pinpoint = expansionHub = false;
    }

    public static boolean isOpen() {
        return open;
    }

    public static boolean isRunning() {
        return running;
    }

    //Blaze runs follower.update() and the drive, so the loop mustn't
    public static boolean drives() {
        return running && pinpoint;
    }

    //Blaze fills the Expansion Hub's bulk data, so don't clear it
    public static boolean feedsExpansionHub() {
        return running && expansionHub;
    }

    public static String status() {

        if (!BlazeConstants.ENABLED) return "off (BlazeConstants.ENABLED)";
        if (!open) return "not started" + (fault.isEmpty() ? "" : ": " + fault);
        String s = (running ? "running" : "ready") + ", drive updates " + driveUpdates + ", mechanism updates " + mechanismUpdates;
        return fault.isEmpty() ? s : s + " | " + fault;
    }

    private static void fail(String what, Throwable t) {

        fault = fault.isEmpty() ? what + ": " + t : fault + "; " + what + ": " + t;
        RobotLog.ee("Blaze", t, what + " failed, carrying on without it");
    }
}
