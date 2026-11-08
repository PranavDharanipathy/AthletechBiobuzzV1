package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (group = "testing")
public class Spin2Servos extends OpMode {

    public static String SERVO1_DEVICE_NAME = "";
    public static String SERVO2_DEVICE_NAME = "";

    public static double SERVO1_POSITION = 0;
    public static double SERVO2_POSITION = 0;

    public static boolean MIRROR = true;

    private Servo servo1, servo2;

    @Override
    public void init() {

        servo1 = hardwareMap.servo.get(SERVO1_DEVICE_NAME);
        servo2 = hardwareMap.servo.get(SERVO2_DEVICE_NAME);
    }

    public void loop() {

        servo1.setPosition(SERVO1_POSITION);
        servo2.setPosition(MIRROR ? 1d - SERVO1_POSITION : SERVO2_POSITION);

        telemetry.addData("servo1", servo1.getPosition());
        telemetry.addData("servo2", servo2.getPosition());
        telemetry.addData("Mirror", MIRROR);
        telemetry.update();
    }

}
