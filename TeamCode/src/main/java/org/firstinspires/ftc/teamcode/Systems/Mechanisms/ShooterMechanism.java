package org.firstinspires.ftc.teamcode.Systems.Mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;

import dev.nextftc.robot.Mechanism;

/// the whole shooter: flywheel + turret run by Shooter's own logic (goal aim, SOTM, routing), localized off the drive's
/// follower. Periodic aims and holds the flywheel every loop; the drive's periodic has to run first (AthletechRobot's order)
public final class ShooterMechanism implements Mechanism {

    public final Flywheel flywheel;
    public final TurretBase turret;
    public final Shooter shooter = new Shooter();
    private boolean spinUp, started;

    /// turret must point straight at INIT: that's its home
    public ShooterMechanism(HardwareMap hardwareMap, DriveMechanism drive) {

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
        if (started) shooter.updateAutonomous(spinUp);
    }

    public void setSpinUp(boolean spinUp) {
        this.spinUp = spinUp;
    }

    public Command spinUp() {
        return instant(() -> spinUp = true);
    }

    public Command spinDown() {
        return instant(() -> spinUp = false);
    }

    /// aimed, the aim is one it can shoot from, and the flywheel is within band of its target
    public boolean isReady(double flywheelBand) {
        return started && spinUp && turret.isOnTarget() && shooter.isAimSolutionUsable() && Math.abs(flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) <= flywheelBand;
    }

    public void stopNow() {

        started = spinUp = false;
        flywheel.reset();
        turret.setVolts(0, 0);
    }
}
