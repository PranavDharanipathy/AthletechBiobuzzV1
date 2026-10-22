package org.firstinspires.ftc.teamcode.shooter.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.shooter.core.MathUtil;
import org.firstinspires.ftc.teamcode.shooter.core.SysIdFitter;
import org.firstinspires.ftc.teamcode.shooter.hardware.DataLog;
import org.firstinspires.ftc.teamcode.shooter.hardware.ShooterConfig;

import java.util.List;

/**
 * Measures the flywheel model u = kS·sgn(ω) + kV·ω + kA·ω' (step 1 of calibration).
 *
 * <p>Profile (about 34 s): quasistatic voltage ramp 0 → 11 V at 0.5 V/s (gives kS, kV), then 11 V / 4 V steps
 * every 1.5 s (gives kA). The voltage is computed from the MEASURED battery voltage every loop, so the fit
 * is in true volts. The fit uses zero-lag centered differences (SysIdFitter), the same method the off-robot
 * test validates. Copy the printed kS/kV/kA into ShooterConfig. The raw CSV is saved for review.
 *
 * <p>Safety: the flywheel must be free to spin with no ball loaded. Press STOP at any time (coasts down).
 */
@TeleOp(name = "Shooter: Flywheel Characterization", group = "Shooter Tuning")
public final class FlywheelCharacterization extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR);
        motor.setDirection(ShooterConfig.FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        double radPerCount = MathUtil.TAU / ShooterConfig.FLYWHEEL_COUNTS_PER_REV;

        DataLog log = new DataLog(8000, "t", "pos_rad", "volts", "battery");
        telemetry.addLine("Flywheel characterization: ~34 s. Wheel must spin freely, NO ball loaded.");
        telemetry.addLine("Press START. Press STOP any time to abort.");
        telemetry.update();
        waitForStart();

        long t0 = System.nanoTime();
        while (opModeIsActive()) {
            for (LynxModule h : hubs) h.clearBulkCache();
            double t = (System.nanoTime() - t0) * 1e-9;
            if (t > 34) break;
            double volts = t < 22 ? 0.5 * t : ((((int) ((t - 22) / 1.5)) % 2 == 0) ? 11.0 : 4.0);
            double vb = battery.getVoltage();
            double power = MathUtil.clamp(volts / Math.max(vb, 6.0), 0, 1);
            motor.setPower(power);
            log.set(0, t);
            log.set(1, motor.getCurrentPosition() * radPerCount);
            log.set(2, power * vb);
            log.set(3, vb);
            if (!log.commit()) break;
            telemetry.addData("t", "%.1f / 34 s", t);
            telemetry.addData("command", "%.2f V (battery %.2f V)", volts, vb);
            telemetry.update();
        }
        motor.setPower(0);

        SysIdFitter.Result r = SysIdFitter.fit(log.column(0), log.column(1), log.column(2), log.rows(), 4, 20);
        String path = log.save("flywheel_sysid");
        while (!isStopRequested()) {
            telemetry.addLine("=== Flywheel model (paste into ShooterConfig) ===");
            telemetry.addData("result", r.toString());
            if (r.ok) {
                telemetry.addData("FLYWHEEL_KS", "%.4f", r.kS);
                telemetry.addData("FLYWHEEL_KV", "%.5f", r.kV);
                telemetry.addData("FLYWHEEL_KA", "%.5f", r.kA);
                telemetry.addData("free speed @12V", "%.0f rpm", (12 - r.kS) / r.kV * 60 / MathUtil.TAU);
                telemetry.addData("quality", r.rSquared > 0.97 ? "good (R² > 0.97)" : "SUSPECT: check belt slip / encoder / wiring");
            }
            telemetry.addData("log", path);
            telemetry.update();
            sleep(100);
        }
    }
}
