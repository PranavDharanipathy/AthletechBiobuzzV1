package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainConstants {

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(DeviceConstants.leftFrontMotorDeviceName);
                c.backLeftName.set(DeviceConstants.leftBackMotorDeviceName);
                c.frontRightName.set(DeviceConstants.rightFrontMotorDeviceName);
                c.backRightName.set(DeviceConstants.rightBackMotorDeviceName);

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static Follower create(HardwareMap h) {

        return null;
    }
}
