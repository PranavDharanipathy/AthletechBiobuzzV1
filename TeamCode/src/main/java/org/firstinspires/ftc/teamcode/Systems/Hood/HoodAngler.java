package org.firstinspires.ftc.teamcode.Systems.Hood;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

public class HoodAngler {

    private final Servo leftHoodServo;
    private final Servo rightHoodServo;

    public HoodAngler(HardwareMap hardwareMap, String leftHoodAnglerServoName, String rightHoodAnglerServoName) {

        leftHoodServo = new MonitoredServo(hardwareMap, leftHoodAnglerServoName, false);
        rightHoodServo = new MonitoredServo(hardwareMap, rightHoodAnglerServoName, false);

        Servo.Direction leftDirection = HoodConstants.LEFT_HOOD_ANGLER_SERVO_DIRECTION;
        leftHoodServo.setDirection(leftDirection);
        rightHoodServo.setDirection(leftDirection == Servo.Direction.FORWARD ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public Servo.Direction[] getServoDirections() {
        return new Servo.Direction[] {leftHoodServo.getDirection(), rightHoodServo.getDirection()};
    }

    public void setPosition(double position) {
        leftHoodServo.setPosition(position);
        rightHoodServo.setPosition(position);
    }

    public void setSafePosition(double position) {
        setPosition(MathUtil.clamp(position, HoodConstants.HOOD_ANGLER_MAX_POSITION, HoodConstants.HOOD_ANGLER_MIN_POSITION));
    }

    public double getPosition() {
        return leftHoodServo.getPosition();
    }

}
