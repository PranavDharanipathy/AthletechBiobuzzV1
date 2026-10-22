package org.firstinspires.ftc.teamcode.shooter.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.core.HoodModel;
import org.firstinspires.ftc.teamcode.shooter.core.MathUtil;
import org.firstinspires.ftc.teamcode.shooter.hardware.ShooterConfig;

/**
 * Builds the servo-position → launch-angle table for the (nonlinear) hood linkage (step 3 of calibration).
 *
 * <p>Use: dpad up/down moves the hood servo in 0.01 steps (bumpers: 0.002 fine steps). At 5–7 positions
 * spread over the travel, measure the LAUNCH angle (the direction the ball leaves, from horizontal; e.g. a
 * phone inclinometer on the hood's exit surface) and write each pair down. Enter them in
 * HoodModel.Config (servoPositions / launchAnglesDeg). The ballistics study needs about 40°–80° of launch
 * angle; telemetry warns if the end points don't cover it.
 *
 * <p>Both servos move together (the second mirrored). Watch for the linkage binding at the ends, and keep the
 * table inside the travel where the two servos agree.
 */
@TeleOp(name = "Shooter: Hood Calibration", group = "Shooter Tuning")
public final class HoodCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo l = hardwareMap.get(Servo.class, ShooterConfig.HOOD_SERVO_LEFT);
        Servo r = hardwareMap.get(Servo.class, ShooterConfig.HOOD_SERVO_RIGHT);
        HoodModel model = new HoodModel(ShooterConfig.hood());
        double pos = 0.5;
        boolean prevUp = false, prevDown = false, prevLb = false, prevRb = false;

        telemetry.addLine("Hood calibration: dpad ±0.01, bumpers ±0.002. Press START.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !prevUp) pos += 0.01;
            if (gamepad1.dpad_down && !prevDown) pos -= 0.01;
            if (gamepad1.right_bumper && !prevRb) pos += 0.002;
            if (gamepad1.left_bumper && !prevLb) pos -= 0.002;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevRb = gamepad1.right_bumper;
            prevLb = gamepad1.left_bumper;
            pos = MathUtil.clamp(pos, 0, 1);
            l.setPosition(pos);
            r.setPosition(model.mirroredPosition(pos));

            telemetry.addData("servo position", "%.3f (mirror %.3f)", pos, model.mirroredPosition(pos));
            telemetry.addData("current table says", "%.1f° launch angle", Math.toDegrees(model.angleForServoPosition(pos)));
            telemetry.addData("table range", "%.1f°–%.1f° (needed ≈ 40°–80°)",
                    Math.toDegrees(model.minAngle()), Math.toDegrees(model.maxAngle()));
            telemetry.addLine("Measure the launch angle here and record (position, angle).");
            telemetry.update();
        }
    }
}
