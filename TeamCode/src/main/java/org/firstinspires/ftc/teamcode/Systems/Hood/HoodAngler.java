package org.firstinspires.ftc.teamcode.Systems.Hood;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

/**
 * Two mirrored hood servos rigidly locked to one hood. Any mismatch in their neutral angles (spline
 * mounting, a 25T spline tooth is 14.4°, or servo-to-servo PWM calibration) makes them fight: current,
 * heat, and a hood angle set by whichever servo is stronger. Per-servo trims in {@link HoodConstants}
 * take that mismatch out; set them so both servos draw minimal current at rest.
 */
public class HoodAngler {

    private final Servo leftHoodServo;
    private final Servo rightHoodServo;

    public HoodAngler(HardwareMap hardwareMap, String leftHoodAnglerServoName, String rightHoodAnglerServoName) {
        this(new MonitoredServo(hardwareMap, leftHoodAnglerServoName, false),
                new MonitoredServo(hardwareMap, rightHoodAnglerServoName, false));
    }

    /** Device injection keeps the mirroring and trim logic testable without hardware. */
    public HoodAngler(Servo leftHoodServo, Servo rightHoodServo) {

        this.leftHoodServo = leftHoodServo;
        this.rightHoodServo = rightHoodServo;

        Servo.Direction leftDirection = HoodConstants.LEFT_HOOD_ANGLER_SERVO_DIRECTION;
        this.leftHoodServo.setDirection(leftDirection);
        this.rightHoodServo.setDirection(leftDirection == Servo.Direction.FORWARD ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public Servo.Direction[] getServoDirections() {
        return new Servo.Direction[] {leftHoodServo.getDirection(), rightHoodServo.getDirection()};
    }

    /** Commands both servos to the same hood position, each offset by its own trim. */
    public void setPosition(double position) {
        leftHoodServo.setPosition(MathUtil.clamp(position + HoodConstants.LEFT_HOOD_TRIM, 0, 1));
        rightHoodServo.setPosition(MathUtil.clamp(position + HoodConstants.RIGHT_HOOD_TRIM, 0, 1));
    }

    public void setSafePosition(double position) {
        setPosition(MathUtil.clamp(position, HoodConstants.HOOD_ANGLER_MAX_POSITION, HoodConstants.HOOD_ANGLER_MIN_POSITION));
    }

    /** The commanded hood position, without trim. */
    public double getPosition() {
        return leftHoodServo.getPosition() - HoodConstants.LEFT_HOOD_TRIM;
    }

    /** Raw positions written to {left, right}, trims included. */
    public double[] getServoPositions() {
        return new double[] {leftHoodServo.getPosition(), rightHoodServo.getPosition()};
    }

}
