package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DrivetrainConstants {

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(DeviceConstants.leftFrontMotorDeviceName);
                c.backLeftName.set(DeviceConstants.leftBackMotorDeviceName);
                c.frontRightName.set(DeviceConstants.rightFrontMotorDeviceName);
                c.backRightName.set(DeviceConstants.rightBackMotorDeviceName);

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.9680610416442392);
        c.yPodOffset.set(-3.0641817107914);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27362960399191244);
                Controller secondaryTranslationalForward = Controller.proportional(0.10109882715733483);
                Controller primaryTranslationalLateral = Controller.proportional(0.40160942740160216);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14838395222335246);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010621722975782691));
                c.brake.set(Controller.proportionalFeedforward(0.009028464529415288));

                c.headingFeedback.set(Controller.proportional(4.157323895428409));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07099558019121109, 0.0027650306732593505));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11862585636379593, 0.07171955671562538));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001676800674622409, 0.0017600975673743072));

                c.maxAchievableForwardVelocity.set(89.30699536388548);
                c.maxAchievableStrafeVelocity.set(73.03787612574429);
                c.naturalForwardDeceleration.set(22.310600397035557);
                c.naturalStrafeDeceleration.set(54.64917696901708);
            }
    );

    public static Follower create(HardwareMap h) {

        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}
