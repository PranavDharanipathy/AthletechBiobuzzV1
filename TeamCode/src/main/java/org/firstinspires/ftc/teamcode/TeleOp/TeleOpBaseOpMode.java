package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Field.Garden;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utility.AutoHandoff;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.Recording.Recorder;
import org.firstinspires.ftc.teamcode.Utility.Recording.RobotRecording;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.List;
import java.util.Locale;

public abstract class TeleOpBaseOpMode extends OpMode {

    public AllianceState alliance;

    public BetterGamepad controller1;
    public BetterGamepad controller2;

    public Follower follower;

    public DcMotor left_front, right_front, left_back, right_back;

    public GeneralVeloMotor intake;

    public Blocker blocker;

    public FIArm fiArm;

    public TurretBase turret;

    public Flywheel flywheel;

    private List<LynxModule> robotHubs;

    public boolean fromAuto;

    protected Recorder recorder;

    @Override
    public void stop() {

        if (turret != null) turret.saveLanyards();
        if (recorder != null) {
            recorder.sample();
            recorder.close();
        }
        BlazeSession.end();
    }

    public void addDriverTelemetry(final Shooter shooter) {

        telemetry.setAutoClear(false);
        telemetry.addData("Pose", () -> {

            Pose pose = follower.pose();
            return String.format(Locale.US, "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        });
        telemetry.addData("Blaze", BlazeSession::status);
        telemetry.addData("Start", () -> AutoHandoff.lastResult);
        telemetry.addData("Log", () -> recorder != null ? recorder.status() : "off");
        telemetry.addData("Turret", () -> !TurretConstants.TURRET_ENABLED ? "DISABLED (TurretConstants.TURRET_ENABLED)" : turret.isEncoderFrozen() ? "STOPPED: encoder not changing under a hard push (check its cable, or Blaze). Restart the OpMode" : shooter.isTurretLocked() ? "LOCKED at 0 (gamepad 1 Y to aim again)" : "aiming (gamepad 1 Y locks it at 0)");
        telemetry.addData("Relocalize (gamepad 1 PS/guide)", () -> (controller1.curr_main_button ? "button DOWN" : "button up") + ", done " + shooter.getRelocalizations() + "x, to the " + Garden.forAlliance(alliance.getAlliance()) + " GARDEN corner");
    }

    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        }

        BlazeSession.engage(hardwareMap, follower);
    }

    public void clearCacheOfLynxModule() {

        BlazeSession.tick();

        HardwareMonitor.get().update();
        if (recorder != null) recorder.sample();
    }

    public void initializeComponents(Alliance alliance) {

        this.alliance = new AllianceState(alliance);

        //Blaze first: it swaps the hardwareMap's motors
        BlazeSession.begin(hardwareMap);

        left_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftFrontMotorDeviceName, false);
        right_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightFrontMotorDeviceName, false);
        left_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftBackMotorDeviceName, false);
        right_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightBackMotorDeviceName, false);

        follower = DrivetrainConstants.createFollower(hardwareMap);

        intake = new GeneralVeloMotor(hardwareMap, DeviceConstants.intakeMotorDeviceName);

        blocker = new Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));

        fiArm = new FIArm(
                new MonitoredServo(hardwareMap, DeviceConstants.fiArmLeftServoDeviceName, false),
                new MonitoredServo(hardwareMap, DeviceConstants.fiArmRightServoDeviceName, false)
        );

        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));

        fromAuto = AutoHandoff.take();
        turret = fromAuto ? new TurretBase(hardwareMap, AutoHandoff.turretTicks) : new TurretBase(hardwareMap);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);

        recorder = RobotRecording.open(getClass().getSimpleName());
        RobotRecording.context(recorder, getClass().getSimpleName(), alliance.toString(), hardwareMap);
        RobotRecording.loop(recorder);
        RobotRecording.gamepad(recorder, "gamepad1", controller1);
        RobotRecording.gamepad(recorder, "gamepad2", controller2);
        RobotRecording.pose(recorder, follower);
        RobotRecording.turret(recorder, turret);
        RobotRecording.flywheel(recorder, flywheel);
        RobotRecording.veloMotor(recorder, "intake_motor", intake);
        RobotRecording.fiArm(recorder, fiArm);
    }

    protected void record(Shooter shooter, Blocker blocker, Intake intake, Drivetrain drivetrain) {

        RobotRecording.shooter(recorder, shooter);
        RobotRecording.blocker(recorder, blocker != null ? blocker : this.blocker, blocker != null);
        RobotRecording.intake(recorder, intake);
        RobotRecording.drivetrain(recorder, drivetrain);
    }

    public Drivetrain createDrivetrain() {
        return new Drivetrain(left_front, right_front, left_back, right_back, follower, controller1, flywheel.getVoltageSensor());
    }

    public void applyComponentTraits() {

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setPose(fromAuto ? AutoHandoff.pose() : FieldConstants.START_POSE);

        intake.setVelocityPDFCoefficients(
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[0],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[1],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]
        );

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
    }

}
