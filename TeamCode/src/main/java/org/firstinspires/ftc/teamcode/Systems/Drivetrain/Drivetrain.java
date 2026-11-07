package org.firstinspires.ftc.teamcode.Systems.Drivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.QuadraticCurve;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

/// TeleOp drive with pose hold (DriveController): robot-centric sticks through the QuadraticCurve and ratios, and the robot held
/// to where the sticks say in all three axes, so a defender can't shove it, turn it or speed it up. Call follower.update()
/// first each loop: the hold runs on its pose. Gamepad 1 back turns the hold off (sticks straight to power) and on again
public class Drivetrain {

    private final DcMotor leftFront, rightFront, leftBack, rightBack;
    private final Follower follower;
    private final BetterGamepad controller1;
    private final VoltageSensor voltageSensor;
    private final VoltageCompensator voltage = new VoltageCompensator(DrivetrainConstants.DRIVE_VOLTAGE_SAMPLE_PERIOD, DrivetrainConstants.DRIVE_VOLTAGE_FILTER_TAU, 12);

    private final DriveController controller = new DriveController();
    private final QuadraticCurve curve = new QuadraticCurve();
    private final double[] command = new double[3], wheels = new double[4];
    private double forwardRatio = 1, strafeRatio = 1, turnRatio = 1;
    private boolean holdEnabled = DrivetrainConstants.DRIVE_HOLD_ENABLED, holding;
    private double lastTime = Double.NaN;

    /// motors with directions and BRAKE already set (TeleOpBaseOpMode.applyComponentTraits)
    public Drivetrain(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, Follower follower, BetterGamepad controller1, VoltageSensor voltageSensor) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.follower = follower;
        this.controller1 = controller1;
        this.voltageSensor = voltageSensor;
    }

    public void setRatios(double forward, double strafe, double turn) {

        forwardRatio = forward;
        strafeRatio = strafe;
        turnRatio = turn;
    }

    public void setHoldEnabled(boolean enabled) {
        holdEnabled = enabled;
    }

    /// the hold is on and has a pose it trusts
    public boolean isHolding() {
        return holding;
    }

    public DriveController getController() {
        return controller;
    }

    public double[] getCommand() {
        return command;
    }

    /// dashboard edits to DrivetrainConstants land on the next update
    public void applyConstants() {

        for (int a = 0; a < 3; a++) {
            controller.kS[a] = DrivetrainConstants.DRIVE_KS[a];
            controller.kV[a] = DrivetrainConstants.DRIVE_KV[a];
            controller.kA[a] = DrivetrainConstants.DRIVE_KA[a];
            controller.maxAccel[a] = DrivetrainConstants.DRIVE_MAX_ACCEL[a] > 0 ? DrivetrainConstants.DRIVE_MAX_ACCEL[a] : Double.POSITIVE_INFINITY;
            controller.maxDecel[a] = DrivetrainConstants.DRIVE_MAX_DECEL[a] > 0 ? DrivetrainConstants.DRIVE_MAX_DECEL[a] : Double.POSITIVE_INFINITY;
            controller.bandwidth[a] = DrivetrainConstants.DRIVE_BANDWIDTH[a];
            controller.maxIntegral[a] = DrivetrainConstants.DRIVE_MAX_INTEGRAL_POWER;
        }
        controller.damping = DrivetrainConstants.DRIVE_DAMPING;
        controller.integralRatio = DrivetrainConstants.DRIVE_INTEGRAL_RATIO;
        controller.leash[DriveController.FORWARD] = controller.leash[DriveController.LEFT] = DrivetrainConstants.DRIVE_LEASH_INCHES;
        controller.leash[DriveController.TURN] = Math.toRadians(DrivetrainConstants.DRIVE_LEASH_DEGREES);
        controller.accelFraction = DrivetrainConstants.DRIVE_ACCEL_FRACTION;
    }

    public void update() {

        if (controller1.backHasJustBeenPressed) holdEnabled = !holdEnabled;
        //robot frame: forward, left, ccw (the right stick turns clockwise)
        command[DriveController.FORWARD] = forwardRatio * shape(-controller1.left_stick_y());
        command[DriveController.LEFT] = -strafeRatio * shape(controller1.left_stick_x());
        command[DriveController.TURN] = -turnRatio * shape(controller1.right_stick_x());
        drive(command);
    }

    /// command: forward, left, ccw, -1..1, already shaped
    public void drive(double[] robotCommand) {

        double now = Clock.seconds();
        double dt = Double.isNaN(lastTime) ? 0.02 : now - lastTime;
        lastTime = now;
        if (voltageSensor != null && voltage.shouldSample(now)) voltage.addSample(voltageSensor.getVoltage(), now);
        applyConstants();
        holding = false;
        if (holdEnabled) {
            Pose pose = follower.pose();
            Velocity velocity = follower.velocity();
            holding = controller.update(dt, robotCommand, pose.x(), pose.y(), pose.heading(), velocity.vx, velocity.vy, velocity.omega, voltage.volts());
        }
        else controller.reset();
        if (holding) DriveController.wheels(controller.power[0], controller.power[1], controller.power[2], wheels);
        else DriveController.wheels(robotCommand[0], robotCommand[1], robotCommand[2], wheels);
        leftFront.setPower(wheels[0]);
        rightFront.setPower(wheels[1]);
        leftBack.setPower(wheels[2]);
        rightBack.setPower(wheels[3]);
    }

    private double shape(double stick) {

        if (Math.abs(stick) <= GeneralConstants.JOYSTICK_MINIMUM) return 0;
        return curve.getOutput(stick);
    }
}
