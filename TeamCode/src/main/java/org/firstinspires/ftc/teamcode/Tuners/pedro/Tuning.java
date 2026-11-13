package org.firstinspires.ftc.teamcode.Tuners.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.Tuners.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;

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
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, DrivetrainConstants.localizerConfig), Tuning::blazeMecanum);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Tuning::blazeMecanum, null, null);
    }

    //Blaze before the motors are got, it swaps them in the hardwareMap; MonitorOpModeHook closes it
    private static Mecanum blazeMecanum(HardwareMap hardwareMap) {

        BlazeSession.begin(hardwareMap);
        BlazeSession.tick();
        return new Mecanum(hardwareMap, DrivetrainConstants.driveConfig);
    }
}
