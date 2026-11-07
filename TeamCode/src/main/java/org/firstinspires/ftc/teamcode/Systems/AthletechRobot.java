package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Systems.Mechanisms.BlockerMechanism;
import org.firstinspires.ftc.teamcode.Systems.Mechanisms.DriveMechanism;
import org.firstinspires.ftc.teamcode.Systems.Mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.Systems.Mechanisms.ShooterMechanism;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/// NextFTC builds this once at app start, so the constructor can't touch hardware: each NextOpMode calls init.
/// Every loop: the OpMode's periodic, then this periodic (fresh bulk reads), then the mechanisms in order (drive first,
/// so the shooter aims off this loop's pose), then the commands
public final class AthletechRobot implements NextRobot {

    public DriveMechanism drive;
    public IntakeMechanism intake;
    public BlockerMechanism blocker;
    public ShooterMechanism shooter;

    private final Set<Mechanism> mechanisms = new LinkedHashSet<>();
    private List<LynxModule> hubs = Collections.emptyList();

    public AthletechRobot() {}

    public void init(HardwareMap hardwareMap, Pose start) {
        init(hardwareMap, start, false);
    }

    public void init(HardwareMap hardwareMap, Pose start, boolean reverseStrafe) {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        clearCaches();
        drive = new DriveMechanism(hardwareMap, start, reverseStrafe);
        intake = new IntakeMechanism(hardwareMap);
        blocker = new BlockerMechanism(hardwareMap);
        shooter = new ShooterMechanism(hardwareMap, drive);
        mechanisms.clear();
        mechanisms.add(drive);
        mechanisms.add(intake);
        mechanisms.add(blocker);
        mechanisms.add(shooter);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    @Override
    public void periodic() {

        clearCaches();
        HardwareMonitor.get().update();
    }

    private void clearCaches() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    /// everything safe: flywheel and turret off, blocker shut, intake and drive stopped
    public void stopAll() {

        if (blocker != null) blocker.set(false);
        if (intake != null) intake.stopNow();
        if (shooter != null) shooter.stopNow();
        if (drive != null) drive.stop();
    }
}
