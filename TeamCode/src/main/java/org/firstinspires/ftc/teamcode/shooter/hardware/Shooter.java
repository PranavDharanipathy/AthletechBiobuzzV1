package org.firstinspires.ftc.teamcode.shooter.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.shooter.core.AimSolver;
import org.firstinspires.ftc.teamcode.shooter.core.Ballistics;
import org.firstinspires.ftc.teamcode.shooter.core.FlywheelController;
import org.firstinspires.ftc.teamcode.shooter.core.HiveTarget;
import org.firstinspires.ftc.teamcode.shooter.core.HoodModel;
import org.firstinspires.ftc.teamcode.shooter.core.MathUtil;
import org.firstinspires.ftc.teamcode.shooter.core.RobotState;
import org.firstinspires.ftc.teamcode.shooter.core.ShooterCore;
import org.firstinspires.ftc.teamcode.shooter.core.ShotTable;
import org.firstinspires.ftc.teamcode.shooter.core.TurretController;
import org.firstinspires.ftc.teamcode.shooter.core.VoltageCompensator;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.List;

/**
 * Shooter subsystem: hardware I/O around {@link ShooterCore}.
 *
 * <p>Per loop: {@link #update(RobotState)} reads sensors (from the bulk cache), runs the core, and writes
 * outputs only when they changed (each hub write is an I/O transaction). The battery voltage is read at
 * 20 Hz, not every loop. The shot table is loaded from a cache file in milliseconds, or built on a background
 * thread at init (shooting reports NO_SOLUTION until it's ready).
 *
 * <p>Bulk caching: this class sets MANUAL mode and clears the cache once per loop, unless the OpMode says
 * it manages caching itself ({@code manageBulkCache = false}).
 */
public final class Shooter {

    private final DcMotorEx flywheelMotor;
    private final CRServo turretLeft, turretRight;
    private final Servo hoodLeft, hoodRight;
    private final TurretAngleSensor turretSensor;
    private final VoltageSensor battery;
    private final List<LynxModule> hubs;
    private final boolean manageBulkCache;

    private final VoltageCompensator voltage;
    private final ShooterCore core;
    private final Ballistics ballistics;
    private volatile ShotTable table;
    private volatile String tableStatus = "not loaded";
    private final double radPerFlywheelCount;
    private final long startNanos = System.nanoTime();

    // write caches
    private double lastFlywheelPower = Double.NaN, lastTurretL = Double.NaN, lastTurretR = Double.NaN;
    private double lastHood = Double.NaN;

    private double loopDt, lastLoopTime = Double.NaN;

    public Shooter(HardwareMap hw, HiveTarget.Alliance alliance, boolean manageBulkCache) {
        this.manageBulkCache = manageBulkCache;
        hubs = hw.getAll(LynxModule.class);
        if (manageBulkCache) {
            for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        flywheelMotor = hw.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR);
        flywheelMotor.setDirection(ShooterConfig.FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our controller, not the hub's
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast: never brake a flywheel

        turretLeft = hw.get(CRServo.class, ShooterConfig.TURRET_SERVO_LEFT);
        turretRight = hw.get(CRServo.class, ShooterConfig.TURRET_SERVO_RIGHT);
        turretLeft.setDirection(ShooterConfig.TURRET_LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turretRight.setDirection(ShooterConfig.TURRET_RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        hoodLeft = hw.get(Servo.class, ShooterConfig.HOOD_SERVO_LEFT);
        hoodRight = hw.get(Servo.class, ShooterConfig.HOOD_SERVO_RIGHT);

        turretSensor = TurretAngleSensor.fromConfig(hw);

        battery = hw.voltageSensor.iterator().next();
        voltage = new VoltageCompensator(ShooterConfig.VOLTAGE_SAMPLE_PERIOD, ShooterConfig.VOLTAGE_FILTER_TAU,
                ShooterConfig.NOMINAL_VOLTS);
        radPerFlywheelCount = MathUtil.TAU / ShooterConfig.FLYWHEEL_COUNTS_PER_REV;

        ballistics = new Ballistics(ShooterConfig.ball());
        HoodModel hood = new HoodModel(ShooterConfig.hood());
        core = new ShooterCore(ShooterConfig.core(), new FlywheelController(ShooterConfig.flywheel()),
                new TurretController(ShooterConfig.turret()), hood, new AimSolver(ShooterConfig.aim()),
                new HiveTarget(ShooterConfig.hive(), alliance, HiveTarget.UpCell.AUDIENCE_SIDE), null);
        startTableLoad(hood);
    }

    // ------------------------------------------------------------------ shot table cache

    private void startTableLoad(HoodModel hood) {
        AimSolver.Config ac = ShooterConfig.aim();
        HiveTarget.Config hc = ShooterConfig.hive();
        ShooterCore.Config sc = ShooterConfig.core();
        final double dz = (hc.openingCenterZ - ac.exitHeight) * MathUtil.IN_TO_M;
        final double exitH = ac.exitHeight * MathUtil.IN_TO_M;
        final double maxExit = sc.exitEfficiency * sc.wheelRadiusM * sc.maxFlywheelSpeed;
        final ShotTable.Config tc = ShooterConfig.shotTable();
        final double hMin = hood.minAngle(), hMax = hood.maxAngle();
        final long sig = ShotTable.signature(ballistics.params(), tc, dz, exitH, hMin, hMax, maxExit);
        final File file = new File(AppUtil.FIRST_FOLDER, "shooter_shot_table.bin");

        Thread th = new Thread(() -> {
            try {
                if (file.exists()) {
                    try (DataInputStream in = new DataInputStream(new BufferedInputStream(new FileInputStream(file)))) {
                        ShotTable t = ShotTable.read(in, tc, sig);
                        if (t != null) {
                            install(t, "loaded from cache");
                            return;
                        }
                    }
                }
                tableStatus = "building (parameters changed or no cache)...";
                long t0 = System.nanoTime();
                ShotTable t = new ShotTable(ballistics, tc, dz, exitH, hMin, hMax, maxExit);
                try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(file)))) {
                    t.write(out, sig);
                }
                install(t, String.format(java.util.Locale.US, "built in %.1f s and cached", (System.nanoTime() - t0) * 1e-9));
            } catch (Exception e) {
                tableStatus = "ERROR: " + e.getMessage();
            }
        }, "ShotTableLoader");
        th.setPriority(Thread.MIN_PRIORITY);
        th.setDaemon(true);
        th.start();
    }

    private void install(ShotTable t, String status) {
        table = t;
        tableStatus = status + (t.hasAnySolution()
                ? String.format(java.util.Locale.US, " | range %.2f–%.2f m", t.minValidDistance(), t.maxValidDistance())
                : " | NO SOLUTIONS: check hood range / target height");
    }

    public boolean isTableReady() { return table != null; }
    public String tableStatus() { return tableStatus; }

    // ------------------------------------------------------------------ loop

    public double time() { return (System.nanoTime() - startNanos) * 1e-9; }

    /** Runs one shooter loop with the latest localization. */
    public void update(RobotState robot) {
        if (manageBulkCache) for (LynxModule h : hubs) h.clearBulkCache();
        double t = time();
        loopDt = Double.isNaN(lastLoopTime) ? 0 : t - lastLoopTime;
        lastLoopTime = t;

        if (voltage.shouldSample(t)) voltage.addSample(battery.getVoltage(), t);

        ShotTable tbl = table;
        core.setShotTable(tbl);
        ShooterCore.Mode requested = core.mode();
        if (tbl == null && requested == ShooterCore.Mode.TRACK) core.setMode(ShooterCore.Mode.OFF);

        double flyPos = flywheelMotor.getCurrentPosition() * radPerFlywheelCount;
        double turretAngle = turretSensor.readAngle();
        core.update(robot, flyPos, turretAngle, voltage.volts(), t);
        core.setMode(requested);

        writeFlywheel(voltage.toPower(core.flywheelVolts()));
        writeTurret(core.turretLeftPower(), core.turretRightPower());
        double hp = core.hoodServoPosition();
        if (!Double.isNaN(hp) && (Double.isNaN(lastHood) || Math.abs(hp - lastHood) > 1e-4)) {
            hoodLeft.setPosition(hp);
            hoodRight.setPosition(core.hood.mirroredPosition(hp));
            lastHood = hp;
        }
    }

    private void writeFlywheel(double p) {
        if (Double.isNaN(lastFlywheelPower) || Math.abs(p - lastFlywheelPower) > 0.002 || (p == 0 && lastFlywheelPower != 0)) {
            flywheelMotor.setPower(p);
            lastFlywheelPower = p;
        }
    }

    private void writeTurret(double l, double r) {
        if (Double.isNaN(lastTurretL) || Math.abs(l - lastTurretL) > 0.002 || (l == 0 && lastTurretL != 0)) {
            turretLeft.setPower(l);
            lastTurretL = l;
        }
        if (Double.isNaN(lastTurretR) || Math.abs(r - lastTurretR) > 0.002 || (r == 0 && lastTurretR != 0)) {
            turretRight.setPower(r);
            lastTurretR = r;
        }
    }

    /** Immediately stops all shooter actuators (call from stop()). */
    public void stop() {
        core.setMode(ShooterCore.Mode.OFF);
        flywheelMotor.setPower(0);
        turretLeft.setPower(0);
        turretRight.setPower(0);
        lastFlywheelPower = lastTurretL = lastTurretR = 0;
    }

    // ------------------------------------------------------------------ commands

    public void setTracking(boolean on) { core.setMode(on ? ShooterCore.Mode.TRACK : ShooterCore.Mode.OFF); }
    public boolean canFire() { return core.canFire(); }
    /** Call when the feeder releases a ball (enables shot anticipation and energy banking). */
    public void notifyFired() { core.notifyFired(); }
    /** The HIVE tipped: aim at the other cell. */
    public void notifyHiveTipped() { core.target.notifyTip(); }
    public void setUpCell(HiveTarget.UpCell c) { core.target.setUpCell(c); }
    public void rehomeTurret(double angleRad) { turretSensor.setCurrentAngle(angleRad); core.turret.clearFault(); }

    public ShooterCore core() { return core; }
    public double batteryVolts() { return voltage.volts(); }
    public double loopDt() { return loopDt; }

    /** Compact telemetry for drivers and tuning (no allocations beyond the SDK's own formatting). */
    public void addTelemetry(Telemetry tm) {
        FlywheelController fw = core.flywheel;
        TurretController tc = core.turret;
        AimSolver.Aim aim = core.aim();
        tm.addData("Shooter", "%s  | table: %s", core.readiness(), tableStatus);
        tm.addData("Flywheel", "%.0f / %.0f rpm  %.1f V  dist %.0f  shots %d%s",
                fw.velocity() * 60 / MathUtil.TAU, fw.target() * 60 / MathUtil.TAU, fw.appliedVolts(),
                fw.disturbance(), fw.shotCount(), fw.isStalled() ? "  STALL FAULT" : "");
        tm.addData("Turret", "%.1f° → %.1f°  err %.2f° (tol %.2f°)%s%s",
                Math.toDegrees(tc.angle()), Math.toDegrees(tc.routedTarget()), Math.toDegrees(tc.error()),
                Math.toDegrees(aim.tolerance), tc.isParked() ? " parked" : "", tc.isFaulted() ? "  FAULT" : "");
        tm.addData("Hood", "%.1f° cmd, est %.1f°", Math.toDegrees(core.hood.commandedAngle()),
                Math.toDegrees(core.hood.estimatedAngle()));
        tm.addData("Aim", "d=%.2f m  tof=%.2f s  cell=%s", aim.distanceM, aim.shot.tof, core.target.upCell());
        tm.addData("Power", "batt %.2f V  loop %.1f ms", voltage.volts(), loopDt * 1000);
    }
}
