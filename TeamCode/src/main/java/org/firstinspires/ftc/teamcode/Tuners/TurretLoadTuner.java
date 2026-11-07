package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

@Config
@TeleOp (group = "tuning")
public class TurretLoadTuner extends TeleOpBaseOpMode {

    public static double TARGET_OFFSET_DEGREES = 0;

    public static double TOLERANCE_DEGREES = 1;

    public static double SETTLE_SECONDS = 0.75;

    public static double FLYWHEEL_VELOCITY = 1450;

    private static final double VOLTAGE_READ_PERIOD = 0.05;

    private static final class Stats {
        int count = 0;
        double errorSquares = 0, worstError = 0;
        int offTargetCount = 0;
        double velocitySquares = 0;
        double powerSquares = 0;
        int reversals = 0;
        double lastPowerSign = 0;
        double voltSum = 0, minVolts = Double.POSITIVE_INFINITY;
        int voltCount = 0;
        double recordedSeconds = 0;
        double lastRecordTime = Double.NaN;
    }

    private final Stats on = new Stats();
    private final Stats off = new Stats();

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private RobotCentricDrive robotCentricDrive;
    private Telemetry telemetry;

    private boolean compensation = TurretConstants.TURRET_VOLTAGE_COMPENSATION;
    private boolean finished = false;
    private double ignoreUntil = 0;
    private double lastTarget = Double.NaN;
    private double lastVoltageRead = Double.NEGATIVE_INFINITY;
    private double lastVolts = Double.NaN;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeComponents(Alliance.BLUE);
        applyComponentTraits();

        robotCentricDrive = new RobotCentricDrive(left_front, right_front, left_back, right_back, controller1);
        intake.provideComponents(new Intake(super.intake), controller1);
        blocker.provideComponents(super.blocker, controller1);

        setUpLynxModule();

        telemetry.addLine("Gamepad 1 drives like in a match. Gamepad 2: A = compensation on/off, BACK = finish.");
        telemetry.update();
    }

    @Override
    public void start() {

        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        ignoreUntil = seconds() + SETTLE_SECONDS;
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        double now = seconds();

        if (!finished && controller2.aHasJustBeenPressed) {
            compensation = !compensation;
            ignoreUntil = now + SETTLE_SECONDS;
        }
        turret.setVoltageCompensation(compensation);

        double offset = MathUtil.clamp(TARGET_OFFSET_DEGREES, ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES);
        double target = turret.startPosition + offset * ShooterConstants.TURRET_TICKS_PER_DEGREE;
        if (target != lastTarget) {
            lastTarget = target;
            ignoreUntil = now + SETTLE_SECONDS;
        }
        turret.setPosition(target);

        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);

        blocker.update();
        follower.update();
        intake.update();
        flywheel.update();
        turret.update();
        robotCentricDrive.update();

        if (now - lastVoltageRead >= VOLTAGE_READ_PERIOD) {
            lastVoltageRead = now;
            lastVolts = turret.getVoltageSensor().getVoltage();
        }

        if (!finished) {
            if (now >= ignoreUntil) record(compensation ? on : off, now);
            if (controller2.backHasJustBeenPressed) finish();
        }

        show();
    }

    private void record(Stats s, double now) {

        double errorDegrees = turret.getError() / ShooterConstants.TURRET_TICKS_PER_DEGREE;
        s.count++;
        s.errorSquares += errorDegrees * errorDegrees;
        s.worstError = Math.max(s.worstError, Math.abs(errorDegrees));
        if (Math.abs(errorDegrees) > TOLERANCE_DEGREES) s.offTargetCount++;

        double velocityDegrees = turret.getVelocity() / ShooterConstants.TURRET_TICKS_PER_DEGREE;
        s.velocitySquares += velocityDegrees * velocityDegrees;

        double power = turret.getPower();
        s.powerSquares += power * power;
        double sign = Math.signum(power);
        if (sign != 0) {
            if (s.lastPowerSign != 0 && sign != s.lastPowerSign) s.reversals++;
            s.lastPowerSign = sign;
        }

        if (MathUtil.isFinite(lastVolts)) {
            s.voltSum += lastVolts;
            s.voltCount++;
            s.minVolts = Math.min(s.minVolts, lastVolts);
        }

        if (MathUtil.isFinite(s.lastRecordTime) && now - s.lastRecordTime < 0.25) {
            s.recordedSeconds += now - s.lastRecordTime;
        }
        s.lastRecordTime = now;
    }

    private void finish() {

        finished = true;
        log("on", on);
        log("off", off);
    }

    private void log(String name, Stats s) {

        if (s.count == 0) return;
        RobotLog.ii("TurretLoadTuner", "compensation=%s samples=%d rmsErrDeg=%.3f worstErrDeg=%.3f offTargetPct=%.1f rmsVelDegPerS=%.2f rmsPower=%.3f reversalsPerS=%.2f avgV=%.2f minV=%.2f", name, s.count, Math.sqrt(s.errorSquares / s.count), s.worstError, 100.0 * s.offTargetCount / s.count, Math.sqrt(s.velocitySquares / s.count), Math.sqrt(s.powerSquares / s.count), reversalsPerSecond(s), avgVolts(s), s.minVolts);
    }

    private void show() {

        if (!finished) {
            telemetry.addLine("Recording. Gamepad 2: A = compensation on/off, BACK = finish.");
            telemetry.addData("Voltage compensation", compensation ? "ON" : "OFF");
            telemetry.addData("Recording", seconds() >= ignoreUntil ? "yes" : "settling");
            telemetry.addData("Error (deg)", "%.2f", turret.getError() / ShooterConstants.TURRET_TICKS_PER_DEGREE);
            telemetry.addData("Turret hub voltage", "%.2f", lastVolts);
            telemetry.addData("Samples on / off", "%d / %d", on.count, off.count);
            telemetry.update();
            return;
        }

        telemetry.addLine("Turret under load, ON vs OFF (lower is steadier):");
        row("RMS error (deg)", on.count == 0 ? Double.NaN : Math.sqrt(on.errorSquares / on.count), off.count == 0 ? Double.NaN : Math.sqrt(off.errorSquares / off.count), "%.3f");
        row("Worst error (deg)", on.count == 0 ? Double.NaN : on.worstError, off.count == 0 ? Double.NaN : off.worstError, "%.2f");
        row("Time off target (%)", pct(on), pct(off), "%.1f");
        row("Shake, RMS speed (deg/s)", on.count == 0 ? Double.NaN : Math.sqrt(on.velocitySquares / on.count), off.count == 0 ? Double.NaN : Math.sqrt(off.velocitySquares / off.count), "%.2f");
        row("Power reversals per s", reversalsPerSecond(on), reversalsPerSecond(off), "%.2f");
        row("RMS power", on.count == 0 ? Double.NaN : Math.sqrt(on.powerSquares / on.count), off.count == 0 ? Double.NaN : Math.sqrt(off.powerSquares / off.count), "%.3f");
        row("Average hub voltage", avgVolts(on), avgVolts(off), "%.2f");
        row("Lowest hub voltage", on.voltCount == 0 ? Double.NaN : on.minVolts, off.voltCount == 0 ? Double.NaN : off.minVolts, "%.2f");
        telemetry.addData("Samples on / off", "%d / %d", on.count, off.count);
        if (on.count == 0 || off.count == 0) {
            telemetry.addLine("Only one setting was recorded. Press A during the run to record both.");
        }
        else if (Math.abs(avgVolts(on) - avgVolts(off)) > 0.3) {
            telemetry.addLine("The battery voltage differed between the two, so the comparison is less fair.");
        }
        telemetry.update();
    }

    private void row(String name, double onValue, double offValue, String format) {
        telemetry.addData(name, "ON " + format + "   OFF " + format, onValue, offValue);
    }

    private static double pct(Stats s) {
        return s.count == 0 ? Double.NaN : 100.0 * s.offTargetCount / s.count;
    }

    private static double avgVolts(Stats s) {
        return s.voltCount == 0 ? Double.NaN : s.voltSum / s.voltCount;
    }

    private static double reversalsPerSecond(Stats s) {
        return s.recordedSeconds <= 0 ? Double.NaN : s.reversals / s.recordedSeconds;
    }

    private static double seconds() {
        return System.nanoTime() * 1e-9;
    }
}
