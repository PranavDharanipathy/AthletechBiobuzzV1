package org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.FastSigmoidCurve;

public class RobotCentricDrive {

    private DcMotor left_front, right_front, left_back, right_back;

    private BetterGamepad controller1;

    //sticks go through the curve (on their size, sign kept), then get scaled by these
    private final FastSigmoidCurve curve = new FastSigmoidCurve();
    private double forwardRatio = 1, strafeRatio = 1, turnRatio = 1;

    public RobotCentricDrive(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.controller1 = controller1;
    }

    public void setRatios(double forward, double strafe, double turn) {

        forwardRatio = forward;
        strafeRatio = strafe;
        turnRatio = turn;
    }

    public FastSigmoidCurve getCurve() {
        return curve;
    }

    public void update() {

        double forward = forwardRatio * shape(-controller1.left_stick_y());
        double strafe = strafeRatio * shape(controller1.left_stick_x());
        double turn = turnRatio * shape(controller1.right_stick_x());

        //mecanum: scale all four down together so none asks for more than full power, keeping the mix
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        double lfPower = (forward + turn + strafe) / denominator;
        double lbPower = (forward + turn - strafe) / denominator;
        double rfPower = (forward - turn - strafe) / denominator;
        double rbPower = (forward - turn + strafe) / denominator;

        left_front.setPower(lfPower);
        right_front.setPower(rfPower);
        left_back.setPower(lbPower);
        right_back.setPower(rbPower);
    }

    private double shape(double stick) {

        double size = Math.abs(stick);
        if (size <= GeneralConstants.JOYSTICK_MINIMUM) return 0;
        return Math.signum(stick) * Math.max(0, Math.min(1, curve.getOutput(Math.min(size, 1))));
    }
}
