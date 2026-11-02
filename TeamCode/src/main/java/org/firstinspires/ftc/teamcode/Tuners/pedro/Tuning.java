package org.firstinspires.ftc.teamcode.Tuners.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.Tests;

public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, DrivetrainConstants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, DrivetrainConstants.driveConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, DrivetrainConstants.driveConfig), null, null);
    }

}
