package org.firstinspires.ftc.teamcode.shooter.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.shooter.core.MathUtil;
import org.firstinspires.ftc.teamcode.shooter.core.SysIdFitter;
import org.firstinspires.ftc.teamcode.shooter.hardware.DataLog;
import org.firstinspires.ftc.teamcode.shooter.hardware.ShooterConfig;
import org.firstinspires.ftc.teamcode.shooter.hardware.TurretAngleSensor;

import java.util.List;

/**
 * Measures the turret model u = kS·sgn(ω) + kV·ω + kA·ω' and the breakaway power (step 2 of calibration).
 *
 * <ol>
 *   <li>Direction check: a short +0.25 pulse must turn the turret CCW (angle increasing). If not, fix
 *       TURRET_*_REVERSED / TURRET_ENCODER_SIGN first (the OpMode stops and says so).</li>
 *   <li>Breakaway: power ramps slowly from 0 until the turret moves, in both directions. Gives the
 *       deadband + static friction (an upper bound for kS).</li>
 *   <li>Sweeps: constant-power runs at 0.2…0.8 back and forth between ±75°. The acceleration phase gives kA,
 *       the cruise phase gives kV.</li>
 * </ol>
 * Place the turret facing forward (0°) with ±90° free before START. Abort: STOP (servos off).
 */
@TeleOp(name = "Shooter: Turret Characterization", group = "Shooter Tuning")
public final class TurretCharacterization extends LinearOpMode {

    private static final double LIMIT = Math.toRadians(75), ABORT = Math.toRadians(100);
    private List<LynxModule> hubs;
    private TurretAngleSensor sensor;
    private CRServo left, right;
    private long t0;
    private final DataLog log = new DataLog(12000, "t", "angle_rad", "power");
    private boolean aborted;

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        left = hardwareMap.get(CRServo.class, ShooterConfig.TURRET_SERVO_LEFT);
        right = hardwareMap.get(CRServo.class, ShooterConfig.TURRET_SERVO_RIGHT);
        left.setDirection(ShooterConfig.TURRET_LEFT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(ShooterConfig.TURRET_RIGHT_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        sensor = TurretAngleSensor.fromConfig(hardwareMap);

        telemetry.addLine("Turret characterization. Turret at 0° (forward), ±90° free, cables slack.");
        telemetry.update();
        waitForStart();
        t0 = System.nanoTime();

        // 1) direction check
        double a0 = read();
        drive(0.25, 0.25);
        double a1 = read();
        drive(0, 0.3);
        if (Math.abs(a1 - a0) < Math.toRadians(2)) {
            finish("Turret did not move at power 0.25: check wiring, servo mode (must be continuous) and the encoder.");
            return;
        }
        if (a1 < a0) {
            finish("Positive power turned the turret CW (angle decreased). Flip TURRET_*_REVERSED or TURRET_ENCODER_SIGN.");
            return;
        }

        // 2) breakaway in both directions
        double breakPos = breakaway(+1), breakNeg = breakaway(-1);

        // 3) sweeps
        for (double p : new double[]{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}) {
            if (!opModeIsActive() || aborted) break;
            sweep(p, +1);
            sweep(p, -1);
        }
        setPower(0);
        if (aborted) {
            finish("ABORTED: turret beyond ±100° (sensor sign or runaway). Check TURRET_ENCODER_SIGN.");
            return;
        }

        SysIdFitter.Result r = SysIdFitter.fit(log.column(0), log.column(1), log.column(2), log.rows(), 3, Math.toRadians(5));
        String path = log.save("turret_sysid");
        while (!isStopRequested()) {
            telemetry.addLine("=== Turret model (paste into ShooterConfig) ===");
            telemetry.addData("result", r.toString());
            telemetry.addData("breakaway", "+%.3f / -%.3f power", breakPos, breakNeg);
            if (r.ok) {
                telemetry.addData("TURRET_KS", "%.4f", r.kS);
                telemetry.addData("TURRET_KV", "%.4f", r.kV);
                telemetry.addData("TURRET_KA", "%.5f", r.kA);
                telemetry.addData("free speed", "%.0f °/s", Math.toDegrees((1 - r.kS) / r.kV));
                telemetry.addData("asymmetry", Math.abs(breakPos - breakNeg) > 0.03
                        ? "LARGE: servos fighting / binding? check mesh and that both servos are reversed correctly" : "ok");
            }
            telemetry.addData("log", path);
            telemetry.update();
            sleep(100);
        }
    }

    private double now() { return (System.nanoTime() - t0) * 1e-9; }

    private double read() {
        for (LynxModule h : hubs) h.clearBulkCache();
        return sensor.readAngle();
    }

    private void setPower(double p) {
        left.setPower(p);
        right.setPower(p);
    }

    private boolean record(double angle, double p) {
        if (Math.abs(angle) > ABORT || Double.isNaN(angle)) {
            aborted = true;
            setPower(0);
            return false;
        }
        log.set(0, now());
        log.set(1, angle);
        log.set(2, p);
        return log.commit();
    }

    /** Holds power p for duration s while logging. */
    private void drive(double p, double s) {
        setPower(p);
        double end = now() + s;
        while (opModeIsActive() && now() < end) if (!record(read(), p)) return;
    }

    private double breakaway(int dir) {
        double start = read();
        double p = 0;
        double tStart = now();
        while (opModeIsActive() && !aborted) {
            p = 0.04 * (now() - tStart); // 0.04 power per second
            setPower(dir * p);
            double a = read();
            record(a, dir * p);
            if (Math.abs(a - start) > Math.toRadians(1.0) || p > 0.5) break;
        }
        setPower(0);
        sleep(400);
        // return to centre gently
        returnToCenter();
        return p;
    }

    private void returnToCenter() {
        double tEnd = now() + 3;
        while (opModeIsActive() && now() < tEnd) {
            double a = read();
            if (Math.abs(a) < Math.toRadians(2)) break;
            setPower(-Math.signum(a) * 0.2);
        }
        setPower(0);
        sleep(300);
    }

    private void sweep(double p, int dir) {
        double target = dir * LIMIT;
        setPower(dir * p);
        double tEnd = now() + 3;
        while (opModeIsActive() && now() < tEnd) {
            double a = read();
            if (!record(a, dir * p)) return;
            if (dir > 0 ? a >= target : a <= target) break;
        }
        setPower(0);
        double settle = now() + 0.3;
        while (opModeIsActive() && now() < settle) if (!record(read(), 0)) return;
    }

    private void finish(String msg) {
        setPower(0);
        String path = log.save("turret_sysid_aborted");
        while (!isStopRequested()) {
            telemetry.addLine(msg);
            telemetry.addData("log", path);
            telemetry.update();
            sleep(100);
        }
    }
}
