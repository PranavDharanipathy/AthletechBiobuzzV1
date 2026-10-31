package org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;

public class RobotCentricDrive {

    private DcMotor left_front, right_front, left_back, right_back;

    private BetterGamepad controller1;

    public RobotCentricDrive(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.controller1 = controller1;
    }

    public void update() {

        double lfPower = -controller1.left_stick_y() + controller1.right_stick_x() + controller1.left_stick_x();
        double lbPower = -controller1.left_stick_y() + controller1.right_stick_x() - controller1.left_stick_x();
        double rfPower = -controller1.left_stick_y() - controller1.right_stick_x() - controller1.left_stick_x();
        double rbPower = -controller1.left_stick_y() - controller1.right_stick_x() + controller1.left_stick_x();

        left_front.setPower(Math.abs(lfPower) > GeneralConstants.JOYSTICK_MINIMUM ? lfPower : 0);
        right_front.setPower(Math.abs(rfPower) > GeneralConstants.JOYSTICK_MINIMUM ? rfPower : 0);
        left_back.setPower(Math.abs(lbPower) > GeneralConstants.JOYSTICK_MINIMUM ? lbPower : 0);
        right_back.setPower(Math.abs(rbPower) > GeneralConstants.JOYSTICK_MINIMUM ? rbPower : 0);
    }
}
