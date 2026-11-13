package org.firstinspires.ftc.teamcode.Systems.Intake;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.Subsystem;

public class FIArm extends Subsystem {

    private boolean isSubsystem;

    private Object lFIArm, rFIArm;

    private BetterGamepad controller1;

    public FIArm(Servo lFIArm, Servo rFIArm) {

        isSubsystem = false;

        this.lFIArm = lFIArm;
        this.rFIArm = rFIArm;
    }
    public FIArm() {}

    public FIArm asSubsystem() {

        isSubsystem = true;
        return this;
    }

    public enum FIArmState {
        DEPLOY(IntakeConstants.FIARM_DEPLOY), RETRACT(IntakeConstants.FIARM_RETRACT);

        private double[] positions;

        FIArmState(double[] positions) {
            this.positions = positions;
        }

        public double getLeftPosition() {
            return positions[0];
        }

        public double getRightPosition() {
            return positions[1];
        }
    }

    private FIArmState state;

    public void setState(FIArmState state) {

        if (this.state == state) return;

        this.state = state;

        if (isSubsystem) {

            ((FIArm) lFIArm).setState(state);
        }
        else {
            ((Servo) lFIArm).setPosition(this.state.getLeftPosition());
            ((Servo) rFIArm).setPosition(this.state.getRightPosition());
        }
    }

    public FIArmState getState() {
        return state;
    }

    public void provideComponents(FIArm fiArm, BetterGamepad controller1) {

        this.lFIArm = fiArm;

        this.controller1 = controller1;
    }

    @Override
    public void update() {
        setState(controller1.left_bumper() ? FIArmState.DEPLOY : FIArmState.RETRACT);
    }
}
