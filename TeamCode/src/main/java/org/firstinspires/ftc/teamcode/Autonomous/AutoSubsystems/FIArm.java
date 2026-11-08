package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;

import dev.nextftc.robot.Mechanism;

/// flower-intake arm. RETRACT is sent at INIT in every auto, until an auto deploys it
public final class FIArm implements Mechanism {

    public final org.firstinspires.ftc.teamcode.Systems.Intake.FIArm arm;

    public FIArm(HardwareMap hardwareMap) {

        arm = new org.firstinspires.ftc.teamcode.Systems.Intake.FIArm(new MonitoredServo(hardwareMap, DeviceConstants.fiArmLeftServoDeviceName, false), new MonitoredServo(hardwareMap, DeviceConstants.fiArmRightServoDeviceName, false));
        set(false);
    }

    public void set(boolean deployed) {
        arm.setState(deployed ? org.firstinspires.ftc.teamcode.Systems.Intake.FIArm.FIArmState.DEPLOY : org.firstinspires.ftc.teamcode.Systems.Intake.FIArm.FIArmState.RETRACT);
    }

    public Command deploy() {
        return instant(() -> set(true));
    }

    public Command retract() {
        return instant(() -> set(false));
    }

    public boolean isDeployed() {
        return arm.getState() == org.firstinspires.ftc.teamcode.Systems.Intake.FIArm.FIArmState.DEPLOY;
    }
}
