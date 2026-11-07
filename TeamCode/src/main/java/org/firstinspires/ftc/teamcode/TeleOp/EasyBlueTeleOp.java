package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (group = "A_Match")
public class EasyBlueTeleOp extends BlueTeleOp {

    @Override
    public void init() {

        super.init();
        robotCentricDrive.setRatios(0.85, 0.85, 0.75);
    }
}
