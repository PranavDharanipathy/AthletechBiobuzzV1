package org.firstinspires.ftc.teamcode.shooter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.core.HiveTarget;
import org.firstinspires.ftc.teamcode.shooter.core.RobotState;
import org.firstinspires.ftc.teamcode.shooter.core.ShooterCore;
import org.firstinspires.ftc.teamcode.shooter.hardware.DataLog;
import org.firstinspires.ftc.teamcode.shooter.hardware.Shooter;

/**
 * Full shooter on a stationary robot with a VIRTUAL pose (no localization needed). It's for end-to-end checks
 * and shot calibration (step 4).
 *
 * <p>The robot is assumed to sit straight "south" of the target cell at the distance you set, so the turret
 * should point at the real cell when the robot faces it. Place the robot at that measured distance from the
 * opening centre.
 * <ul>
 *   <li>A: tracking on/off (spins up, aims, sets the hood)</li>
 *   <li>dpad up/down: distance ±0.10 m (bumpers ±0.02 m)</li>
 *   <li>dpad left/right: exit efficiency η ∓/± 0.005. Shots land SHORT → the real exit speed is lower than
 *       modelled → LOWER η (the flywheel then spins faster). Shots LONG → raise η.</li>
 *   <li>right trigger: tell the controller a ball was fed (for anticipation), then feed it by hand/feeder</li>
 *   <li>X: the HIVE tipped (switch cells)</li>
 * </ul>
 * Every loop is logged (readiness, speeds, angles) to /sdcard/FIRST/shooter_logs/ for later analysis.
 */
@TeleOp(name = "Shooter: Tuning (virtual pose)", group = "Shooter Tuning")
public final class ShooterTuning extends LinearOpMode {

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap, HiveTarget.Alliance.RED, true);
        RobotState rs = new RobotState();
        DataLog log = new DataLog(60000, "t", "ready", "fly_w", "fly_target", "fly_volts", "turret", "turret_target",
                "hood_cmd", "dist_m", "battery");
        double distance = 2.0;
        boolean tracking = false;
        boolean pA = false, pUp = false, pDown = false, pL = false, pR = false, pLb = false, pRb = false, pX = false, pTrig = false;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Shot table", shooter.tableStatus());
            telemetry.addLine("Waiting for START (the table can finish building in the background).");
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            if (gamepad1.a && !pA) { tracking = !tracking; shooter.setTracking(tracking); }
            if (gamepad1.dpad_up && !pUp) distance += 0.10;
            if (gamepad1.dpad_down && !pDown) distance -= 0.10;
            if (gamepad1.right_bumper && !pRb) distance += 0.02;
            if (gamepad1.left_bumper && !pLb) distance -= 0.02;
            if (gamepad1.dpad_right && !pR) shooter.core().config().exitEfficiency += 0.005;
            if (gamepad1.dpad_left && !pL) shooter.core().config().exitEfficiency -= 0.005;
            if (gamepad1.x && !pX) shooter.notifyHiveTipped();
            boolean trig = gamepad1.right_trigger > 0.5;
            if (trig && !pTrig) shooter.notifyFired();
            pA = gamepad1.a; pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down; pRb = gamepad1.right_bumper;
            pLb = gamepad1.left_bumper; pR = gamepad1.dpad_right; pL = gamepad1.dpad_left; pX = gamepad1.x; pTrig = trig;
            distance = Math.max(0.5, distance);

            // Virtual pose: robot faces +y, directly "south" of the target, at `distance` (m) from the opening
            // (horizontal), measured from the turret axis.
            ShooterCore core = shooter.core();
            double dIn = distance / 0.0254;
            double turretForward = core.aimSolver.config().turretForward;
            double heading = Math.PI / 2;
            rs.set(core.target.x(), core.target.y() - dIn - turretForward, heading, 0, 0, 0, shooter.time());

            shooter.update(rs);

            log.set(0, shooter.time());
            log.set(1, core.readiness().ordinal());
            log.set(2, core.flywheel.velocity());
            log.set(3, core.flywheel.target());
            log.set(4, core.flywheel.appliedVolts());
            log.set(5, core.turret.angle());
            log.set(6, core.turret.routedTarget());
            log.set(7, core.hood.commandedAngle());
            log.set(8, core.aim().distanceM);
            log.set(9, shooter.batteryVolts());
            log.commit();

            telemetry.addData("Distance (virtual)", "%.2f m  [dpad ↑↓, bumpers fine]", distance);
            telemetry.addData("Exit efficiency η", "%.3f  [dpad ←→]  short shots → LOWER η",
                    core.config().exitEfficiency);
            telemetry.addData("Ready", core.canFire() ? ">>> FIRE <<<" : core.readiness().toString());
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }
        shooter.stop();
        log.save("shooter_tuning");
    }
}
