package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Hood.Hood;
import org.firstinspires.ftc.teamcode.Systems.Hood.HoodAngler;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.List;

public abstract class TeleOpBaseOpMode extends OpMode {

    public TeleOpBaseOpMode() {}

    public BetterGamepad controller1;
    public BetterGamepad controller2;

    public DcMotor left_front, right_front, left_back, right_back;

    public GeneralVeloMotor intakeMotor;
    public Intake intake;

    public Blocker blocker;

    public TurretBase turret;

    public HoodAngler hoodAngler;
    public Hood hood;

    public Flywheel flywheel;

    private List<LynxModule> robotHubs;

    /// initializes/creates LynxModule
    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        }
    }

    /// Clears cache of LynxModule
    public void clearCacheOfLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.clearBulkCache();
        }

        HardwareMonitor.get().update();
    }

    /// Initializing devices
    public void initializeDevices() {

        left_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftFrontMotorDeviceName, false);
        right_front = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightFrontMotorDeviceName, false);
        left_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.leftBackMotorDeviceName, false);
        right_back = new MonitoredDcMotor(hardwareMap, DeviceConstants.rightBackMotorDeviceName, false);

        intakeMotor = new GeneralVeloMotor(hardwareMap, DeviceConstants.intakeMotorDeviceName);
        intake = new Intake(intakeMotor);

        blocker = new Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));

        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));

        turret = new TurretBase(hardwareMap);

        hoodAngler = new HoodAngler(hardwareMap, DeviceConstants.hoodAnglerServoDeviceName);
        hood = new Hood(hoodAngler);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setVelocityPDFCoefficients(
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[0],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[1],
                IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]
        );

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        turret.setPositionalCoefficients(TurretConstants.TURRET_POSITIONAL_COEFFICIENTS);
        //turret.reverse();

        hoodAngler.setSafePosition(HoodConstants.HOOD_ANGLER_MAX_POSITION);

    }

}
