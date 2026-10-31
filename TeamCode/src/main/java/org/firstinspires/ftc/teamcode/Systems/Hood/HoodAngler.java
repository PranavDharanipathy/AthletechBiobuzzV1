package org.firstinspires.ftc.teamcode.Systems.Hood;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

public class HoodAngler {

    private final Servo hoodServo;

    public HoodAngler(HardwareMap hardwareMap, String hoodAnglerServoName) {

        hoodServo = hardwareMap.get(Servo.class, hoodAnglerServoName);
        hoodServo.setDirection(HoodConstants.HOOD_ANGLER_SERVO_DIRECTION);
    }

    public Servo.Direction getServoDirection() {
        return hoodServo.getDirection();
    }

    public void setPosition(double position) {
        hoodServo.setPosition(position);
    }

    public void setSafePosition(double position) {
        hoodServo.setPosition(MathUtil.clamp(position, HoodConstants.HOOD_ANGLER_MAX_POSITION, HoodConstants.HOOD_ANGLER_MIN_POSITION));
    }

    public double getPosition() {
        return hoodServo.getPosition();
    }

}
