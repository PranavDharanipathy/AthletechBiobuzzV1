package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredVoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

@Config
@TeleOp (group = "tuning")
public class FlywheelLoadTuner extends TeleOpBaseOpMode {

    public static double TARGET_VELOCITY = 1450;

    public static double SETTLED_MARGIN = FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR;

    private static final int MAX_SAMPLES = 40000;

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();

    private Drivetrain drivetrain;
    private VoltageSensor flywheelHubVoltage;
    private Telemetry telemetry;

    private final double[] times = new double[MAX_SAMPLES];
    private final double[] volts = new double[MAX_SAMPLES];
    private int count = 0;
    private double readSeconds = 0;

    private boolean settled = false;
    private boolean finished = false;
    private int errorCount = 0;
    private double errorSum = 0, errorSquares = 0;
    private double worstOver = 0, worstUnder = 0;
    private int overCount = 0;
    private double estimateNoiseSquares = 0;
    private double loopMsSum = 0, worstLoopMs = 0;
    private int loopCount = 0;

    private FlywheelVoltageFilterFit.Result fit;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeComponents(Alliance.BLUE);
        applyComponentTraits();

        drivetrain = createDrivetrain();

        intake.provideComponents(new Intake(super.intake), controller1);
        blocker.provideComponents(super.blocker, controller1);

        flywheelHubVoltage = new MonitoredVoltageSensor(flywheel.getVoltageSensor(), "flywheelLoadTuner", false);

        record(null, blocker, intake, drivetrain);
        setUpLynxModule();

        telemetry.addLine("Drive and run everything like in a match. Press BACK on gamepad 1 to finish.");
        telemetry.update();
    }

    @Override
    public void start() {
        flywheel.setVelocity(TARGET_VELOCITY, true);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        if (!finished) {

            double readStart = seconds();
            double voltage = flywheelHubVoltage.getVoltage();
            double now = seconds();
            readSeconds += now - readStart;
            times[count] = now;
            volts[count] = voltage;
            count++;
        }

        flywheel.setVelocity(TARGET_VELOCITY, true);

        blocker.update();
        follower.update();
        intake.update();
        flywheel.update();
        drivetrain.update();

        if (!finished) {
            record();
            if (controller1.backHasJustBeenPressed || count >= MAX_SAMPLES) finish();
        }

        show();
    }

    private void record() {

        double error = flywheel.getCurrentVelocity() - TARGET_VELOCITY;
        if (!settled && Math.abs(error) <= SETTLED_MARGIN) settled = true;
        if (!settled) return;

        errorCount++;
        errorSum += error;
        errorSquares += error * error;
        worstOver = Math.max(worstOver, error);
        worstUnder = Math.min(worstUnder, error);
        if (error > SETTLED_MARGIN) overCount++;

        double noise = flywheel.getVelocityEstimate() - flywheel.getCurrentVelocity();
        estimateNoiseSquares += noise * noise;

        double loopMs = HardwareMonitor.get().getLastLoopMs();
        if (Double.isFinite(loopMs)) {
            loopMsSum += loopMs;
            loopCount++;
            worstLoopMs = Math.max(worstLoopMs, loopMs);
        }
    }

    private void finish() {

        finished = true;
        if (count > 0) {
            fit = FlywheelVoltageFilterFit.fit(times, volts, count, readSeconds / count, FlywheelConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD, FlywheelConstants.FLYWHEEL_VOLTAGE_FILTER_TAU);
        }

        if (errorCount > 0) {
            RobotLog.ii("FlywheelLoadTuner", "target=%.0f meanError=%.1f rmsError=%.1f worstOver=%.1f worstUnder=%.1f overPct=%.1f oldNoiseRms=%.1f loopMs=%.1f/%.1f", TARGET_VELOCITY, errorSum / errorCount, Math.sqrt(errorSquares / errorCount), worstOver, worstUnder, 100.0 * overCount / errorCount, Math.sqrt(estimateNoiseSquares / errorCount), loopCount == 0 ? Double.NaN : loopMsSum / loopCount, worstLoopMs);
        }
        if (fit != null) {
            RobotLog.ii("FlywheelLoadTuner", "samplePeriod=%.3f tau=%.3f rms=%.4f currentRms=%.4f sag=%.2f", fit.samplePeriod, fit.filterTau, fit.rmsVolts, fit.currentRmsVolts, fit.sagVolts);
        }
    }

    private void show() {

        if (!finished) {
            telemetry.addLine("Recording. Press BACK on gamepad 1 to finish.");
            telemetry.addData("Flywheel speed (hub)", "%.0f", flywheel.getCurrentVelocity());
            telemetry.addData("Flywheel speed (old, loop-timed)", "%.0f", flywheel.getVelocityEstimate());
            telemetry.addData("Flywheel hub voltage", "%.2f", volts[Math.max(0, count - 1)]);
            telemetry.addData("Settled", settled);
            telemetry.update();
            return;
        }

        if (errorCount == 0) {
            telemetry.addLine("The flywheel never got within the settled margin, so there are no speed results.");
        }
        else {
            telemetry.addLine("Flywheel speed under load (ticks/s, + is too fast):");
            telemetry.addData("Average error", "%.1f", errorSum / errorCount);
            telemetry.addData("RMS error", "%.1f", Math.sqrt(errorSquares / errorCount));
            telemetry.addData("Worst too fast", "%.0f", worstOver);
            telemetry.addData("Worst too slow", "%.0f", worstUnder);
            telemetry.addData("Time too fast (%)", "%.1f", 100.0 * overCount / errorCount);
            telemetry.addData("Old speed measurement noise (RMS)", "%.0f", Math.sqrt(estimateNoiseSquares / errorCount));
            telemetry.addData("Loop ms (average / worst)", "%.1f / %.1f", loopCount == 0 ? Double.NaN : loopMsSum / loopCount, worstLoopMs);
        }

        telemetry.addLine();
        if (fit == null) {
            telemetry.addLine("Not enough data for voltage filter settings (run at least 5 s).");
        }
        else {
            telemetry.addLine("Voltage filter for this load, copy into FlywheelConstants:");
            telemetry.addData("FLYWHEEL_VOLTAGE_SAMPLE_PERIOD", "%.3f", fit.samplePeriod);
            telemetry.addData("FLYWHEEL_VOLTAGE_FILTER_TAU", "%.3f", fit.filterTau);
            telemetry.addData("Tracking error, new / current (V)", "%.4f / %.4f", fit.rmsVolts, fit.currentRmsVolts);
            telemetry.addData("Battery sag seen (V)", "%.2f", fit.sagVolts);
            if (!fit.isTrusted()) telemetry.addLine("WARNING: the battery barely sagged, so don't trust these.");
        }
        telemetry.update();
    }

    private static double seconds() {
        return System.nanoTime() * 1e-9;
    }
}
