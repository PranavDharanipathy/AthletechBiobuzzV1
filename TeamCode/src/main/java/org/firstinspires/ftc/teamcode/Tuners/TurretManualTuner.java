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
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

@Config
@TeleOp(name = "Turret Manual Tuner", group = "tuning")
public class TurretManualTuner extends OpMode {

    public static double TARGET_DEGREES = 0;
    public static double STEP_DEGREES = 15;
    public static boolean SINE = false;
    public static double SINE_AMPLITUDE_DEGREES = 25, SINE_HZ = 0.5;
    public static long LOOP_TIME = 0;
    public static boolean VOLTAGE_COMPENSATION = TurretConstants.TURRET_VOLTAGE_COMPENSATION;
    public static double FLYWHEEL_VELOCITY = ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY;

    private TurretBase turret;
    private Flywheel flywheel;
    private boolean flywheelOn = false;
    private BetterGamepad controller1;
    private Telemetry telemetry;
    private double sineStart;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        BlazeSession.begin(hardwareMap);
        turret = new TurretBase(hardwareMap);
        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
        controller1 = new BetterGamepad(gamepad1);
        BlazeSession.engage(hardwareMap, null);
        TARGET_DEGREES = 0;
        telemetry.addLine("Turret must point straight when you INIT. dpad left/right = step, dpad down = center, A = sine on/off, B = flywheel on/off.");
        telemetry.addLine("Edit TurretConstants on the dashboard, it applies live.");
        telemetry.addData("match loop time (ms)", TunedConstants.LOOP_TIME_MS);
        telemetry.update();
    }

    @Override
    public void start() {
        sineStart = time;
    }

    @Override
    public void loop() {

        BlazeSession.tick();
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
        TARGET_DEGREES = Math.max(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, Math.min(ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES, TARGET_DEGREES));

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        double degrees = TARGET_DEGREES, degreesPerSecond = 0;
        if (SINE) {
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
        telemetry.addData("position (deg)", (turret.getPosition() - turret.startPosition) / tpd);
        telemetry.addData("error (deg)", turret.getError() / tpd);
        telemetry.addData("velocity (deg/s)", turret.getVelocity() / tpd);
        telemetry.addData("PID speed (deg/s)", c.positionalVelocity / tpd);
        telemetry.addData("p / i / d (V)", "%.3f / %.3f / %.3f", turret.p, turret.i, turret.d);
        telemetry.addData("friction / hold (V)", "%.3f / %.3f", c.s, c.f);
        telemetry.addData("feedforward (V)", "%.3f", turret.v);
        telemetry.addData("sent (V)", "%.3f", turret.getVolts());
        telemetry.addData("power", "%.3f", turret.getPower());
        telemetry.addData("flywheel (B)", flywheelOn ? String.format(java.util.Locale.US, "%.0f / %.0f ticks/s", flywheel.getCurrentVelocity(), FLYWHEEL_VELOCITY) : "off");
        telemetry.addData("on target (aim gate)", turret.isOnTarget());
        if (turret.isEncoderFrozen()) telemetry.addLine("STOPPED: encoder not changing under a push (check its cable). Restart the OpMode");
        telemetry.addData("dividing by (V)", "%.2f", turret.getCompensationVolts());
        telemetry.addData("loop (ms)", "%.1f", 1000 * turret.getLoopDt());
        telemetry.update();

        if (LOOP_TIME > 0) sleep(LOOP_TIME);
    }

    @Override
    public void stop() {

        turret.setVolts(0, 0);
        flywheel.setVelocity(0, true);
        flywheel.update();
        BlazeSession.end();
    }
}
