package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

//edit TurretConstants on the dashboard, it applies live
@Config
@TeleOp(name = "Turret Manual Tuner", group = "tuning")
public class TurretManualTuner extends OpMode {

    public static double TARGET_DEGREES = 0;
    public static double STEP_DEGREES = 15;
    public static boolean SINE = false;
    public static double SINE_AMPLITUDE_DEGREES = 25, SINE_HZ = 0.5;
    public static long LOOP_TIME = 0; //extra ms per loop, set to TunedConstants.LOOP_TIME_MS to act like a match
    public static boolean VOLTAGE_COMPENSATION = TurretConstants.TURRET_VOLTAGE_COMPENSATION;
    public static double FLYWHEEL_VELOCITY = ShooterConstants.DEFAULT_FLYWHEEL_VELOCITY; //B spins it, that's what shakes the turret

    private TurretBase turret;
    private Flywheel flywheel;
    private boolean flywheelOn = false;
    private BetterGamepad controller1;
    private Telemetry telemetry;
    private double sineStart;
    private boolean sweeping = false;
    private double sweepStart;
    private String lanyardNote = "";

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new TurretBase(hardwareMap);
        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
        controller1 = new BetterGamepad(gamepad1);
        TARGET_DEGREES = 0;
        telemetry.addLine("Turret must point straight when you INIT. dpad left/right = step, dpad down = center, A = sine on/off, B = flywheel on/off.");
        telemetry.addLine("Lanyards: X = learning sweep over the whole travel on/off (saves when you stop it), back = forget the map.");
        telemetry.addData("match loop time (ms)", TunedConstants.LOOP_TIME_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        sineStart = time;
    }

    @Override
    public void loop() {

        HardwareMonitor.get().update();
        controller1.getInformation();

        if (controller1.dpad_rightHasJustBeenPressed) TARGET_DEGREES += STEP_DEGREES;
        if (controller1.dpad_leftHasJustBeenPressed) TARGET_DEGREES -= STEP_DEGREES;
        if (controller1.dpad_downHasJustBeenPressed) TARGET_DEGREES = 0;
        if (controller1.bHasJustBeenPressed) flywheelOn = !flywheelOn;
        flywheel.setVelocity(flywheelOn ? FLYWHEEL_VELOCITY : 0, !flywheelOn);
        flywheel.update();
        if (controller1.aHasJustBeenPressed) {
            SINE = !SINE;
            sineStart = time;
        }
        if (controller1.xHasJustBeenPressed) {
            if (sweeping) {
                sweeping = false;
                turret.forceLanyardLearning = false;
                TARGET_DEGREES = 0;
                turret.saveLanyards();
                lanyardNote = "sweep stopped, map saved";
            }
            else if (!TurretConstants.TURRET_LANYARDS_ENABLED) lanyardNote = "turn on TURRET_LANYARDS_ENABLED first";
            else {
                sweeping = true;
                SINE = false;
                sweepStart = time;
                turret.forceLanyardLearning = true;
                lanyardNote = "";
            }
        }
        if (controller1.backHasJustBeenPressed && !sweeping) {
            TurretBase.clearLanyards();
            lanyardNote = "map cleared";
        }
        TARGET_DEGREES = Math.max(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, Math.min(ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES, TARGET_DEGREES));

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        double degrees = TARGET_DEGREES, degreesPerSecond = 0;
        double low = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES + 5, high = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES - 5;
        double speed = TurretConstants.TURRET_LANYARD_SWEEP_DEGREES_PER_SECOND, dwell = TurretConstants.TURRET_LANYARD_SWEEP_DWELL;
        if (sweeping) {
            degrees = TurretLanyardSweep.position(time - sweepStart, low, high, speed, dwell);
            degreesPerSecond = TurretLanyardSweep.velocity(time - sweepStart, low, high, speed, dwell);
        }
        else if (SINE) {
            double w = 2 * Math.PI * SINE_HZ, s = time - sineStart;
            degrees += SINE_AMPLITUDE_DEGREES * Math.sin(w * s);
            degreesPerSecond = SINE_AMPLITUDE_DEGREES * w * Math.cos(w * s);
        }

        turret.setVoltageCompensation(VOLTAGE_COMPENSATION);
        turret.setPosition(turret.startPosition + degrees * tpd);
        turret.setTargetVelocity(degreesPerSecond * tpd);
        turret.update();

        TurretController c = turret.getController();
        telemetry.addData("target (deg)", degrees);
        telemetry.addData("reference (deg)", (turret.getReference() - turret.startPosition) / tpd);
        telemetry.addData("position (deg)", (turret.getPosition() - turret.startPosition) / tpd);
        telemetry.addData("error (deg)", turret.getError() / tpd);
        telemetry.addData("velocity (deg/s)", turret.getVelocity() / tpd);
        telemetry.addData("p", "%.3f", turret.p);
        telemetry.addData("i", "%.3f", turret.i);
        telemetry.addData("d", "%.3f", turret.d);
        telemetry.addData("f", "%.3f", turret.f);
        telemetry.addData("v", "%.3f", turret.v);
        telemetry.addData("power", "%.3f", turret.getPower());
        telemetry.addData("kp / kd / ki", "%.6f / %.6f / %.6f", c.kp, c.kd, c.ki);
        telemetry.addData("flywheel (B)", flywheelOn ? String.format(java.util.Locale.US, "%.0f / %.0f ticks/s", flywheel.getCurrentVelocity(), FLYWHEEL_VELOCITY) : "off");
        telemetry.addData("shake (deg RMS)", "%.3f", turret.getVibrationDegrees());
        telemetry.addData("bandwidth now", "%.2f of %.2f", turret.getEffectiveBandwidth(), TurretConstants.TURRET_BANDWIDTH);
        telemetry.addData("on target (aim gate)", turret.isOnTarget());
        if (TurretConstants.TURRET_LANYARDS_ENABLED) {
            telemetry.addData("lanyard volts now", "%.2f (map's biggest %.2f)", turret.getLanyardVolts(), TurretBase.getLanyardMap().largestVolts());
            if (sweeping) telemetry.addData("lanyard sweep", "%.1f sweeps done, let it do about 8, X stops", (time - sweepStart) / TurretLanyardSweep.period(low, high, speed, dwell));
        }
        else telemetry.addData("lanyards", "off (TURRET_LANYARDS_ENABLED)");
        if (!lanyardNote.isEmpty()) telemetry.addLine(lanyardNote);
        telemetry.addData("dividing by (V)", "%.2f", turret.getCompensationVolts());
        telemetry.addData("loop (ms)", "%.1f", 1000 * turret.getLoopDt());
        telemetry.update();

        if (LOOP_TIME > 0) sleep(LOOP_TIME);
    }

    @Override
    public void stop() {

        turret.saveLanyards();
        turret.setVolts(0, 0);
        flywheel.setVelocity(0, true);
        flywheel.update();
    }
}
