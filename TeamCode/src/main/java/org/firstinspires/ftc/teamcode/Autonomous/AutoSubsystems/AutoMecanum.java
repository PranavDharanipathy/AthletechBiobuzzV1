package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class AutoMecanum extends Mecanum {

    private final boolean reverseStrafe;

    public AutoMecanum(HardwareMap hardwareMap, MecanumConfig config, boolean reverseStrafe) {

        super(hardwareMap, config);
        this.reverseStrafe = reverseStrafe;
    }

    @Override
    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {

        double[] wheels = super.computeWheelPowersUnnormalized(powers);
        //Pedro order: LF, RF, LB, RB. Flip only the strafe term before normalization.
        if (reverseStrafe) {
            double correction = 2 * powers.strafe();
            wheels[0] += correction;
            wheels[1] -= correction;
            wheels[2] -= correction;
            wheels[3] += correction;
        }
        return wheels;
    }
}
