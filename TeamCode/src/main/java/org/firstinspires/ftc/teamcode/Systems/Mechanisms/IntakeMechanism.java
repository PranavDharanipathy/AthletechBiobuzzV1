package org.firstinspires.ftc.teamcode.Systems.Mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;

import dev.nextftc.robot.Mechanism;

public final class IntakeMechanism implements Mechanism {

    public final GeneralVeloMotor motor;
    private double velocity;

    public IntakeMechanism(HardwareMap hardwareMap) {

        motor = new GeneralVeloMotor(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.intakeMotorDeviceName, false));
        motor.getMotor().setPower(0);
        motor.setDirection(IntakeConstants.INTAKE_MOTOR_DIRECTION);
        motor.setVelocityPDFCoefficients(IntakeConstants.INTAKE_PDF_COEFFICIENTS[0], IntakeConstants.INTAKE_PDF_COEFFICIENTS[1], IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]);
    }

    @Override
    public void periodic() {

        if (velocity == 0) {
            motor.getMotor().setPower(0);
            return;
        }
        motor.setVelocity(velocity);
        motor.update();
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public Command run() {
        return instant(() -> velocity = IntakeConstants.INTAKE_VELOCITY);
    }

    public Command stop() {
        return instant(() -> velocity = 0);
    }

    public void stopNow() {

        velocity = 0;
        motor.getMotor().setPower(0);
    }
}
