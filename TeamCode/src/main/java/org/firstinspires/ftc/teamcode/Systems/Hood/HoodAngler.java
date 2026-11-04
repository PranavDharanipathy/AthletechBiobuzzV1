package org.firstinspires.ftc.teamcode.Systems.Hood;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

public class HoodAngler {

    private final Servo hood;

    public HoodAngler(HardwareMap hardwareMap, String hoodAnglerServoName) {

        hood = new MonitoredServo(hardwareMap, hoodAnglerServoName, false);

        Servo.Direction leftDirection = HoodConstants.HOOD_ANGLER_SERVO_DIRECTION;
        hood.setDirection(leftDirection);
    }

    public void setPosition(double position) {
        hood.setPosition(position);
    }

    public void setSafePosition(double position) {
        setPosition(MathUtil.clamp(position, HoodConstants.HOOD_ANGLER_MAX_POSITION, HoodConstants.HOOD_ANGLER_MIN_POSITION));
    }

    public double getPosition() {
        return hood.getPosition();
    }

}
