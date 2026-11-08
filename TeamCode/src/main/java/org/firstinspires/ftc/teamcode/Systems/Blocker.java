package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
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

    //COMPONENT
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

    //SUBSYSTEM
    public void provideComponents(Blocker blocker, BetterGamepad controller1) {

        this.blocker = blocker;

        this.controller1 = controller1;
    }

    private Flywheel flywheel;
    private ShotGate shotGate;

    public void provideComponents(Blocker blocker, BetterGamepad controller1, Flywheel flywheel) {

        provideComponents(blocker, controller1);

        this.flywheel = flywheel;

        shotGate = new ShotGate(
                FlywheelConstants.SHOT_GATE_ENTER_BAND_BELOW, FlywheelConstants.SHOT_GATE_ENTER_BAND_ABOVE,
                FlywheelConstants.SHOT_GATE_HOLD_BAND_BELOW, FlywheelConstants.SHOT_GATE_HOLD_BAND_ABOVE,
                FlywheelConstants.SHOT_GATE_DWELL,
                FlywheelConstants.SHOT_GATE_MIN_OPEN,
                FlywheelConstants.SHOT_GATE_PULSE
        );
    }

    private TurretBase turret;

    public void provideComponents(Blocker blocker, BetterGamepad controller1, Flywheel flywheel, TurretBase turret) {

        provideComponents(blocker, controller1, flywheel);
        this.turret = turret;
    }

    public boolean isAimReady() {
        return turret == null || !TurretConstants.TURRET_AIM_GATE_ENABLED || turret.isOnTarget();
    }

    public boolean isFlywheelReady() {

        if (shotGate == null || !FlywheelConstants.SHOT_GATE_ENABLED) return true;

        return shotGate.isReady();
    }

    @Override
    public void update() {

        if (shotGate != null) {
            shotGate.update(System.nanoTime() * 1e-9, flywheel.getTargetVelocity(), flywheel.getCurrentVelocity(), state == BlockerState.CLEAR);
            flywheel.setFeeding(state == BlockerState.CLEAR);
        }

        //Shooter.shotWillScore: a ball fired at the flywheel's actual speed, with the robot moving as it is, lands in the CELL
        //if (controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD) && isFlywheelReady() && isAimReady() && Shooter.shotWillScore()) {
        if (controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            setState(BlockerState.CLEAR);
        }
        else {
            setState(BlockerState.BLOCK);
        }
    }
}