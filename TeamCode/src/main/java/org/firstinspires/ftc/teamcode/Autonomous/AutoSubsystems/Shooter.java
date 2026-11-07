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

/// the whole shooter: flywheel + turret run by Shooter's own logic (goal aim, SOTM, routing), localized off the drive's
/// follower. Periodic aims and holds the flywheel every loop; the drive's periodic has to run first (AthletechRobot's order)
public final class Shooter implements Mechanism {

    public final Flywheel flywheel;
    public final TurretBase turret;
    public final org.firstinspires.ftc.teamcode.Systems.Shooter shooter = new org.firstinspires.ftc.teamcode.Systems.Shooter();
    private boolean rampUp, started;

    /// turret must point straight at INIT: that's its home
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
        if (started) shooter.updateAutonomous(rampUp);
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

    /// aimed, the aim is one it can shoot from, and the flywheel is within band of its target
    public boolean isReady(double flywheelBand) {
        return started && rampUp && turret.isOnTarget() && shooter.isAimSolutionUsable() && Math.abs(flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) <= flywheelBand;
    }

    public void stopNow() {

        started = rampUp = false;
        flywheel.reset();
        turret.setVolts(0, 0);
    }
}
