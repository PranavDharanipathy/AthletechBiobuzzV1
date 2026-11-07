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
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
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

    public TurretBase turret;

    public Flywheel flywheel;

    private List<LynxModule> robotHubs;

    //keeps what the turret learned about the lanyards for next time
    @Override
    public void stop() {
        if (turret != null) turret.saveLanyards();
    }

    /// pose and the relocalize button on the Driver Station. The lines are made once and only filled in when telemetry
    /// is sent (4 times a second), so nothing is built per loop
    public void addDriverTelemetry(final Shooter shooter) {

        telemetry.setAutoClear(false);
        telemetry.addData("Pose", () -> {

            Pose pose = follower.pose();
            return String.format(Locale.US, "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        });
        telemetry.addData("Relocalize (gamepad 1 PS/guide)", () -> (controller1.curr_main_button ? "button DOWN" : "button up") + ", done " + shooter.getRelocalizations() + "x, to the " + Garden.forAlliance(alliance.getAlliance()) + " GARDEN corner");
    }

    //lynx module setup
    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        }
    }

    //clears lynx cache
    public void clearCacheOfLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.clearBulkCache();
        }

        HardwareMonitor.get().update();
    }

    //creates devices
    public void initializeComponents(Alliance alliance) {

        this.alliance = new AllianceState(alliance);

        left_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftFrontMotorDeviceName, false);
        right_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightFrontMotorDeviceName, false);
        left_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftBackMotorDeviceName, false);
        right_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightBackMotorDeviceName, false);

        follower = DrivetrainConstants.createFollower(hardwareMap);

        intake = new GeneralVeloMotor(hardwareMap, DeviceConstants.intakeMotorDeviceName);

        blocker = new Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));

        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));

        turret = new TurretBase(hardwareMap);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    //sets device traits
    public void applyComponentTraits() {

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setPose(FieldConstants.START_POSE);

        intake.setVelocityPDFCoefficients(
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[0],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[1],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]
        );

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
    }

}
