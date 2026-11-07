package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredVoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.Collections;

@TeleOp (group = "tuning")
public class FlywheelVoltageFilterTuner extends LinearOpMode {

    private static final int CYCLES = 4;
    private static final double SPIN_SECONDS = 3;
    private static final double COAST_SECONDS = 2.5;

    private static final double SPIN_UP_CHECK_SECONDS = 1.5;
    private static final double MIN_SPEED_FRACTION = 0.25;
    private static final double MAX_SPEED_FRACTION = 1.3;

    private static final double SPEED_WINDOW_SECONDS = 0.1;

    private static final int MAX_SAMPLES = 20000;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        BlazeSession.begin(hardwareMap);

        Flywheel flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        VoltageSensor battery = new MonitoredVoltageSensor(flywheel.getVoltageSensor(), "voltageFilterTuner", false);

        double spinVelocity = Collections.max(ShooterConstants.FLYWHEEL_VELOCITIES);

        telemetry.addLine("The flywheel will spin up and coast down on its own.");
        telemetry.addData("Run time (s)", CYCLES * (SPIN_SECONDS + COAST_SECONDS));
        telemetry.addData("Top speed (ticks/s)", spinVelocity);
        telemetry.addLine("Remove all balls and keep clear of the flywheel, then press START.");
        telemetry.update();

        BlazeSession.engage(hardwareMap, null);

        waitForStart();
        if (isStopRequested()) return;

        flywheel.reset();

        double[] times = new double[MAX_SAMPLES];
        double[] volts = new double[MAX_SAMPLES];
        int count = 0;
        double readSeconds = 0;

        String fault = null;
        double start = seconds();
        double cycleLength = SPIN_SECONDS + COAST_SECONDS;
        double runLength = CYCLES * cycleLength;

        double windowStartTime = start;
        int windowStartPosition = flywheel.getEncoder().getCurrentPosition();
        double measuredSpeed = 0;
        double peakSpeed = 0;

        while (opModeIsActive() && count < MAX_SAMPLES) {

            BlazeSession.tick();
            HardwareMonitor.get().update();

            double readStart = seconds();
            double voltage = battery.getVoltage();
            double now = seconds();
            readSeconds += now - readStart;

            times[count] = now;
            volts[count] = voltage;
            count++;

            double elapsed = now - start;
            if (elapsed >= runLength) break;

            double cycleTime = elapsed % cycleLength;
            boolean spinning = cycleTime < SPIN_SECONDS;

            flywheel.setVelocity(spinning ? spinVelocity : 0, true);
            flywheel.update();

            double windowEndTime = seconds();
            if (windowEndTime - windowStartTime >= SPEED_WINDOW_SECONDS) {
                int position = flywheel.getEncoder().getCurrentPosition();
                windowEndTime = seconds();
                measuredSpeed = (position - windowStartPosition) / (windowEndTime - windowStartTime);
                peakSpeed = Math.max(peakSpeed, measuredSpeed);
                windowStartTime = windowEndTime;
                windowStartPosition = position;
            }

            if (spinning) {
                if (measuredSpeed > spinVelocity * MAX_SPEED_FRACTION) {
                    fault = "Flywheel overspeed: " + measuredSpeed + " ticks/s over " + SPEED_WINDOW_SECONDS + " s";
                }
                else if (cycleTime > SPIN_UP_CHECK_SECONDS && measuredSpeed < spinVelocity * MIN_SPEED_FRACTION) {
                    fault = "Flywheel not reaching speed: " + measuredSpeed + " ticks/s. Check the motor, encoder and direction.";
                }
                if (fault != null) break;
            }

            telemetry.addData("Running (s left)", runLength - elapsed);
            telemetry.addData("Raw voltage", voltage);
            telemetry.addData("Flywheel speed (100 ms average)", measuredSpeed);
            telemetry.addData("Flywheel speed (hub)", flywheel.getCurrentVelocity());
            telemetry.update();
        }

        flywheel.reset();

        if (isStopRequested()) return;

        FlywheelVoltageFilterFit.Result result = fault != null || count == 0 ? null : FlywheelVoltageFilterFit.fit(times, volts, count, readSeconds / count, FlywheelConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD, FlywheelConstants.FLYWHEEL_VOLTAGE_FILTER_TAU);

        if (result != null) {
            RobotLog.ii("FlywheelVoltageFilterTuner", "samplePeriod=%.3f tau=%.3f rms=%.4f currentRms=%.4f sag=%.2f readMs=%.2f samples=%d", result.samplePeriod, result.filterTau, result.rmsVolts, result.currentRmsVolts, result.sagVolts, readSeconds / count * 1000, count);
        }

        while (opModeIsActive()) {

            if (fault != null) {
                telemetry.addLine("STOPPED EARLY, no result.");
                telemetry.addLine(fault);
            }
            else if (result == null) {
                telemetry.addLine("Not enough data for a result. Run it again.");
                telemetry.addData("Samples", count);
            }
            else {
                telemetry.addLine("Done! Copy these into FlywheelConstants:");
                telemetry.addData("FLYWHEEL_VOLTAGE_SAMPLE_PERIOD", "%.3f", result.samplePeriod);
                telemetry.addData("FLYWHEEL_VOLTAGE_FILTER_TAU", "%.3f", result.filterTau);
                telemetry.addLine();
                telemetry.addData("Tracking error, new (V)", "%.4f", result.rmsVolts);
                telemetry.addData("Tracking error, current (V)", "%.4f", result.currentRmsVolts);
                telemetry.addData("Battery sag seen (V)", "%.2f", result.sagVolts);
                telemetry.addData("Voltage read time (ms)", "%.2f", readSeconds / count * 1000);
                telemetry.addData("Peak flywheel speed (ticks/s)", "%.0f", peakSpeed);
                telemetry.addData("Samples", count);
                if (!result.isTrusted()) {
                    telemetry.addLine();
                    telemetry.addLine("WARNING: the battery barely sagged, so don't trust this result.");
                }
            }
            telemetry.update();
            sleep(100);
        }
    }

    private static double seconds() {
        return System.nanoTime() * 1e-9;
    }
}
