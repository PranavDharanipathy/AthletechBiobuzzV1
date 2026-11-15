package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;

import dev.nextftc.robot.Mechanism;

public final class Intake implements Mechanism {

    public final GeneralVeloMotor motor;
    private double velocity;

    public Intake(HardwareMap hardwareMap) {

        motor = new GeneralVeloMotor(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.intakeMotorDeviceName, false));
        motor.getMotor().setPower(0);
        motor.setDirection(IntakeConstants.INTAKE_MOTOR_DIRECTION);
        motor.setVelocityPDFCoefficients(IntakeConstants.INTAKE_PDF_COEFFICIENTS[0], IntakeConstants.INTAKE_PDF_COEFFICIENTS[1], IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]);
    }

    @Override
    public void periodic() {

        synchronized (BlazeSession.LOCK) {
            if (velocity == 0) {
                motor.getMotor().setPower(0);
                return;
            }
            motor.setVelocity(velocity);
            if (!BlazeSession.feedsExpansionHub()) motor.update();
        }
    }

    //Blaze: its speed loop on each Expansion Hub read
    public void control() {
        if (velocity != 0) motor.update();
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

        synchronized (BlazeSession.LOCK) {
            velocity = 0;
            motor.getMotor().setPower(0);
        }
    }
}
