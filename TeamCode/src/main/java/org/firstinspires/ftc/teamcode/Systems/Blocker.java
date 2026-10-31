package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.Subsystem;

public class Blocker extends Subsystem {

    private boolean isSubsystem;

    private Object blocker; //given Blocker (non-subsystem) or Servo data

    private BetterGamepad controller1;

    public Blocker(Servo blocker) {

        isSubsystem = false;

        blocker.setDirection(BlockerConstants.BLOCKER_SERVO_DIRECTION);
        this.blocker = blocker;
    }
    public Blocker() {}

    public Blocker asSubsystem() {
        isSubsystem = true;
        return this;
    }

    // COMPONENT
    public enum BlockerState {
        CLEAR(BlockerConstants.BLOCKER_CLEAR_POSITION), BLOCK(BlockerConstants.BLOCKER_BLOCK_POSITION);

        private double position;

        BlockerState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    private BlockerState state;

    public void setState(BlockerState state) {

        if (this.state == state) return;

        this.state = state;

        if (isSubsystem) {

            ((Blocker) blocker).setState(state);
        }
        else {
            ((Servo) blocker).setPosition(this.state.getPosition());
        }
    }

    public BlockerState getState() {
        return state;
    }

    // SUBSYSTEM
    public void provideComponents(Blocker blocker, BetterGamepad controller1) {

        this.blocker = blocker;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        if (controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            setState(BlockerState.CLEAR);
        }
        else {
            setState(BlockerState.BLOCK);
        }
    }
}