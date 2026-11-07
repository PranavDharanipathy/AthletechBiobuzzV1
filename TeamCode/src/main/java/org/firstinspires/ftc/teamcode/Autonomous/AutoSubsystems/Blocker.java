package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;

import dev.nextftc.robot.Mechanism;

public final class Blocker implements Mechanism {

    public final org.firstinspires.ftc.teamcode.Systems.Blocker blocker;

    public Blocker(HardwareMap hardwareMap) {

        blocker = new org.firstinspires.ftc.teamcode.Systems.Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));
        blocker.setState(org.firstinspires.ftc.teamcode.Systems.Blocker.BlockerState.BLOCK);
    }

    public void set(boolean open) {
        blocker.setState(open ? org.firstinspires.ftc.teamcode.Systems.Blocker.BlockerState.CLEAR : org.firstinspires.ftc.teamcode.Systems.Blocker.BlockerState.BLOCK);
    }

    public boolean isOpen() {
        return blocker.getState() == org.firstinspires.ftc.teamcode.Systems.Blocker.BlockerState.CLEAR;
    }

    public Command block() {
        return instant(() -> set(false));
    }

    public Command clear() {
        return instant(() -> set(true));
    }
}
