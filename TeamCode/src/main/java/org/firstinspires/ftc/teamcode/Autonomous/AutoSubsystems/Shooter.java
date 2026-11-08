package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;

import dev.nextftc.robot.Mechanism;

public final class Shooter implements Mechanism {

    public final Flywheel flywheel;
    public final TurretBase turret;
    public final org.firstinspires.ftc.teamcode.Systems.Shooter shooter = new org.firstinspires.ftc.teamcode.Systems.Shooter();
    private boolean rampUp, started, aim = true, locked;

    public Shooter(HardwareMap hardwareMap, Drive drive) {

        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
        flywheel.reset();
        turret = new TurretBase(hardwareMap);
        turret.setVolts(0, 0);
        shooter.provideComponents(flywheel, turret, drive.follower);
    }

    public void start(AllianceState alliance) {

        shooter.start(alliance);
        started = true;
    }

    @Override
    public void periodic() {

        if (started) shooter.updateAutonomous(rampUp, aim);
        else if (locked) {
            turret.setPosition(turret.startPosition);
            turret.setTargetVelocity(0);
            turret.update();
        }
    }

    public void setRampUp(boolean rampUp) {
        this.rampUp = rampUp;
    }

    public Command rampUp() {
        return instant(() -> rampUp = true);
    }

    public Command rampDown() {
        return instant(() -> rampUp = false);
    }

    public Command velocity(double ticksPerSecond) {
        return instant(() -> shooter.manualFlywheelVelocity = ticksPerSecond);
    }

    public void lockHome() {
        locked = true;
    }

    public void home() {

        rampUp = false;
        aim = false;
    }

    public boolean isReady(double flywheelBand) {
        return started && rampUp && turret.isOnTarget() && shooter.isAimSolutionUsable() && Math.abs(flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) <= flywheelBand;
    }

    public void stopNow() {

        started = rampUp = locked = false;
        flywheel.reset();
        turret.setVolts(0, 0);
    }
}
