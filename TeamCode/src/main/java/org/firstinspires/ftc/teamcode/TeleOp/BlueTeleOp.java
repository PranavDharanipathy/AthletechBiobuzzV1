package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;

@TeleOp (group = "A_Match")
public class BlueTeleOp extends TeleOpBaseOpMode {

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Shooter shooter = new Shooter();

    protected Drivetrain drivetrain;

    @Override
    public void init() {

        initializeComponents(Alliance.BLUE);

        applyComponentTraits();

        drivetrain = createDrivetrain();

        intake.provideComponents(new Intake(super.intake, fiArm), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel, turret);
        shooter.provideComponents(flywheel, turret, follower, controller1, controller2);
        addDriverTelemetry(shooter);
        addLoopTimeTelemetry();
        record(shooter, blocker, intake, drivetrain);

        setUpLynxModule();
    }

    @Override
    public void start() {
        shooter.start(alliance);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        synchronized (lock) {
            controller1.getInformation();
            controller2.getInformation();
            shooter.mechanismsElsewhere = intake.motorElsewhere = !mechanismsInLoop();

            blocker.update();
            updateFollower();
            intake.update();
            shooter.update();
            runDrive(drivetrain);
        }
    }

    @Override
    protected boolean usesBlaze() {
        return true;
    }

    //each Pinpoint sample
    @Override
    protected void driveUpdate() {

        drivetrain.driveFromSticks();
        follower.update();
    }

    //each Expansion Hub bulk read: the flywheel, the turret's encoder and the intake are on it
    @Override
    protected void mechanismUpdate() {

        if (!shooter.mechanismsElsewhere) return;
        flywheel.update();
        turret.update();
        super.intake.update();
    }
}
