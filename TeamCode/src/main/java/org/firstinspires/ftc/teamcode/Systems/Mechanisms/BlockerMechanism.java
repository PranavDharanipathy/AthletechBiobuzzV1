package org.firstinspires.ftc.teamcode.Systems.Mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;

import dev.nextftc.robot.Mechanism;

public final class BlockerMechanism implements Mechanism {

    public final Blocker blocker;

    public BlockerMechanism(HardwareMap hardwareMap) {

        blocker = new Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));
        blocker.setState(Blocker.BlockerState.BLOCK);
    }

    public void set(boolean open) {
        blocker.setState(open ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);
    }

    public boolean isOpen() {
        return blocker.getState() == Blocker.BlockerState.CLEAR;
    }

    public Command block() {
        return instant(() -> set(false));
    }

    public Command clear() {
        return instant(() -> set(true));
    }
}
