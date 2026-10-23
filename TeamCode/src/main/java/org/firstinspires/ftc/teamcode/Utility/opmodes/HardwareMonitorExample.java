package org.firstinspires.ftc.teamcode.Utility.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredIMU;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

/**
 * Minimal example of the Monitored* wrappers and the HardwareMonitor. Disabled because the
 * device names below ("leftFront", "claw", "imu") probably differ from your configuration -
 * rename them and remove {@code @Disabled} to try it.
 *
 * After running it, open "Hardware Monitor Viewer" to see the saved session.
 */
@Disabled
@TeleOp(name = "Hardware Monitor Example", group = "Diagnostics")
public class HardwareMonitorExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Optional: let the monitor manage REV bulk caching (encoder/velocity reads become free).
        HardwareMonitor.CONFIG.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

        // Wrappers are drop-in: store them in the normal SDK types. `true` = cache reads per loop.
        DcMotorEx leftFront = new MonitoredDcMotorEx(hardwareMap, "leftFront", true);
        Servo claw = new MonitoredServo(hardwareMap, "claw", false);
        IMU imu = new MonitoredIMU(hardwareMap, "imu", true);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        HardwareMonitor monitor = HardwareMonitor.get();

        waitForStart();
        while (opModeIsActive()) {
            monitor.update(); // first thing every loop

            // Read twice: with caching on, the second read costs nothing.
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingAgain = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            leftFront.setPower(-gamepad1.left_stick_y);
            claw.setPosition(gamepad1.a ? 1.0 : 0.0);

            telemetry.addData("Heading", "%.1f (%.1f)", heading, headingAgain);
            telemetry.addData("Lift current", "%.2f A", leftFront.getCurrent(CurrentUnit.AMPS));
            monitor.addTelemetry(telemetry);
            telemetry.update();
        }
        // Nothing to do here: the session is saved automatically when the OpMode stops.
    }
}
