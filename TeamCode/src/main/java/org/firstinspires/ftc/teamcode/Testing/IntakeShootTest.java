package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;

@Config
@TeleOp(group = "testing")
public class IntakeShootTest extends OpMode {

    public static double INTAKE_POWER = 0;
    public static double FLYWHEEL_POWER = 0;

    private DcMotor intake, flywheel;

    @Override
    public void init() {

        flywheel = hardwareMap.dcMotor.get(DeviceConstants.flywheelMotorDeviceName);
        intake = hardwareMap.dcMotor.get(DeviceConstants.intakeMotorDeviceName);
    }

    @Override
    public void loop() {

        flywheel.setPower(FLYWHEEL_POWER);
        intake.setPower(INTAKE_POWER);
    }
}
