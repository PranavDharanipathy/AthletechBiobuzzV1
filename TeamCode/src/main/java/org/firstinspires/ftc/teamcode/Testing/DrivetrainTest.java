package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;

@TeleOp(group = "testing")
public class DrivetrainTest extends OpMode {

    private RobotCentricDrive drive;

    private BetterGamepad controller1;

    @Override
    public void init() {

        controller1 = new BetterGamepad(gamepad1);

        DcMotor[] motors = {
                hardwareMap.dcMotor.get(DeviceConstants.leftFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.leftBackMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightBackMotorDeviceName)
        };

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new RobotCentricDrive(
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                controller1
        );

    }

    @Override
    public void loop() {

        controller1.getInformation();

        drive.update();

    }
}
