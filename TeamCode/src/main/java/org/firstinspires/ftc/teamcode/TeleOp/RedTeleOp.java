package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;

@TeleOp (group = "A_Match")
public class RedTeleOp extends TeleOpBaseOpMode {

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Shooter shooter = new Shooter();

    protected RobotCentricDrive robotCentricDrive;

    @Override
    public void init() {

        initializeComponents(Alliance.RED);

        applyComponentTraits();

        DcMotor[] motors = {
                hardwareMap.dcMotor.get(DeviceConstants.leftFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.leftBackMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightBackMotorDeviceName)
        };

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        robotCentricDrive = new RobotCentricDrive(
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                controller1
        );

        intake.provideComponents(new Intake(super.intake, fiArm), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel, turret);
        shooter.provideComponents(flywheel, turret, follower, controller1, controller2);
        addDriverTelemetry(shooter);

        setUpLynxModule();
    }

    @Override
    public void start() {
        shooter.start(alliance);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        blocker.update();
        follower.update();
        intake.update();
        shooter.update();
        robotCentricDrive.update();

    }
}
