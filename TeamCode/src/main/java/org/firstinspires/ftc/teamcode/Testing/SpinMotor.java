package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp (group = "testing")
public class SpinMotor extends OpMode {

    public static double MOTOR_POWER = 1;

    public static String motorName = "";

    public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.FORWARD;

    private DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(motorDirection);
    }

    public void loop() {
        motor.setPower(MOTOR_POWER);
    }
}
