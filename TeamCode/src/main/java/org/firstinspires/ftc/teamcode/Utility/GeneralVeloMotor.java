package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class GeneralVeloMotor {

    private DcMotorEx motor;
    public DcMotorEx getMotor() {
        return motor;
    }

    public GeneralVeloMotor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(DcMotorEx.class, deviceName));
    }

    public GeneralVeloMotor(DcMotorEx motor) {

        this.motor = java.util.Objects.requireNonNull(motor, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setVelocityPDFCoefficients(double kp, double kd, double kff) {

        this.kp = kp;
        this.kd = kd;
        this.kff = kff;
    }

    private double targetVelocity, currentVelocity;

    public void setVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public double getCurrent(CurrentUnit currentUnit) {
        return motor.getCurrent(currentUnit);
    }

    private double kp, kd, kff;
    public double p, d, ff;
    private double prevTime = 0, currTime = 0;
    private double prevError = 0, error = 0;

    private double startTime = 0;
    private double getSeconds() {
        return System.nanoTime() * 1e-9 - startTime;
    }

    private boolean firstTick = true;

    public void update() {

        if (firstTick) {

            startTime = getSeconds();
            firstTick = false;
            return;
        }

        prevTime = currTime;
        currTime = getSeconds();

        if (motorActivity == MotorActivity.STOP) {
            motor.setPower(0);
            return;
        }

        double dt = currTime - prevTime;

        currentVelocity = motor.getVelocity();

        prevError = error;
        error = targetVelocity - currentVelocity;

        //proportional
        p = kp * error;

        //derivative
        d = dt > 0 ? kd * (error - prevError) / dt : 0;

        //full feedforward
        ff = kff * targetVelocity;

        double power = p + d + ff;

        motor.setPower(power);
    }

    private MotorActivity motorActivity = MotorActivity.RUN;

    public enum MotorActivity {
        RUN, STOP
    }

    public void setMotorActivity(MotorActivity activity) {
        motorActivity = activity;
    }

    public double getPower() {
        return motor.getPower();
    }
}