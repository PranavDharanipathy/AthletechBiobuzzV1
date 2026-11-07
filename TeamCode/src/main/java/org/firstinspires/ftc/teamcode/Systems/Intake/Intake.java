package org.firstinspires.ftc.teamcode.Systems.Intake;

import static org.firstinspires.ftc.teamcode.Constants.GeneralConstants.TRIGGER_THRESHOLD;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Subsystem;

public class Intake extends Subsystem {

    private boolean isSubsystem;

    private FIArm fiArm; //null = no arm (tuners)

    private Object intake; //given Intake (non-subsystem) or GeneralVeloMotor data

    public Object getIntake() {
        return intake;
    }

    private BetterGamepad controller1;

    public Intake(GeneralVeloMotor intake) {
        this(intake, null);
    }

    public Intake(GeneralVeloMotor intake, FIArm fiArm) {

        isSubsystem = false;

        this.fiArm = fiArm;

        intake.setDirection(IntakeConstants.INTAKE_MOTOR_DIRECTION);
        this.intake = intake;
    }
    public Intake() {}

    public Intake asSubsystem() {

        isSubsystem = true;
        return this;
    }

    //COMPONENT
    public enum IntakeState {
        INTAKE(IntakeConstants.INTAKE_VELOCITY),
        REVERSE_INTAKE(IntakeConstants.REVERSE_INTAKE_VELOCITY),
        IDLE(0);

        private double velocity;

        IntakeState(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    private IntakeState state;

    public void setState(IntakeState state) {

        if (this.state == state) return;

        this.state = state;

        if (isSubsystem) {

            ((Intake) intake).setState(state);
        }
        else {
            ((GeneralVeloMotor) intake).setVelocity(this.state.getVelocity());
        }
    }

    public IntakeState getState() {
        return state;
    }

    public FIArm getFIArm() {
        return fiArm;
    }

    //SUBSYSTEM
    public void provideComponents(Intake intake, BetterGamepad controller1) {

        this.intake = intake;

        this.controller1 = controller1;
    }

    private boolean fiArmState = false;

    @Override
    public void update() {

        if (controller1.right_trigger(TRIGGER_THRESHOLD)) {
            setState(IntakeState.INTAKE);
        }
        else if (controller1.right_bumper()) {
            setState(IntakeState.REVERSE_INTAKE);
        }
        else {
            setState(IntakeState.IDLE);
        }

        FIArm arm = ((Intake) intake).getFIArm();
        if (arm != null) {
            if (controller1.dpad_downHasJustBeenPressed) fiArmState = !fiArmState;
            arm.setState(fiArmState ? FIArm.FIArmState.DEPLOY : FIArm.FIArmState.RETRACT);
        }

        ((GeneralVeloMotor) ((Intake) intake).getIntake()).update();
    }
}