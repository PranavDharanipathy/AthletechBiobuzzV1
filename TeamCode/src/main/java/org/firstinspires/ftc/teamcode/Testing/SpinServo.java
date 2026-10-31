package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (group = "testing")
public class SpinServo extends OpMode {

    public static double SERVO_POSITION = 0;

    public static String servoName = "";

    public static Servo.Direction servoDirection = Servo.Direction.FORWARD;

    private Servo servo;

    @Override
    public void init() {

        servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(servoDirection);
    }

    public void loop() {
        servo.setPosition(SERVO_POSITION);
    }
}