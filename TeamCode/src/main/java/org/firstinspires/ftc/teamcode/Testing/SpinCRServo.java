package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp (group = "testing")
public class SpinCRServo extends OpMode {

    public static double CRSERVO_POWER = 0;

    public static String crservoName = "";

    public static DcMotor.Direction crservoDirection = DcMotor.Direction.FORWARD;

    private CRServo crservo;

    @Override
    public void init() {

        crservo = hardwareMap.get(CRServo.class, crservoName);
        crservo.setDirection(crservoDirection);
    }

    public void loop() {
        crservo.setPower(CRSERVO_POWER);
    }
}
