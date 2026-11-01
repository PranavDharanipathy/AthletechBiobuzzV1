package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp (group = "tuning")
public class HoodInterpolationTuner extends TeleOpBaseOpMode {

    public static boolean SHOOT = false;

    public static double FLYWHEEL_VELOCITY = 0;

    public static double HOOD_POSITION = 0.3;

    public static double DISTANCE_TO_GOAL = 0;

    private final List<Double> distances = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final List<Double> positions = new ArrayList<>();

    private RobotCentricDrive robotCentricDrive;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(10);

        initializeDevices();

        applyComponentTraits();

        robotCentricDrive = new RobotCentricDrive(left_front, right_front, left_back, right_back, controller1);

        //setup lynx module
        setUpLynxModule();
    }

    @Override
    public void start() {
        flywheel.reset();
    }

    private boolean seeHoodInterpolationDataset = false;

    @Override
    public void loop() {

        // clear data at start of loop
        clearCacheOfLynxModule();

        controller1.getInformation();

        if (controller1.yHasJustBeenPressed) {
            seeHoodInterpolationDataset = !seeHoodInterpolationDataset;
        }

        blocker.setState(SHOOT ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);
        intake.setState(SHOOT ? Intake.IntakeState.INTAKE : Intake.IntakeState.IDLE);
        flywheel.setVelocity(FLYWHEEL_VELOCITY, true);

        intakeMotor.update();
        flywheel.update(getBulkReadNanos());

        if (controller1.dpad_upHasJustBeenPressed) {
            distances.add(DISTANCE_TO_GOAL);
            positions.add(HOOD_POSITION);
            velocities.add(flywheel.getTargetVelocity());
        }
        else if (controller1.dpad_downHasJustBeenPressed && !distances.isEmpty()) {

            int lastIndex = distances.size() - 1;

            distances.remove(lastIndex);
            positions.remove(lastIndex);
            velocities.remove(lastIndex);
        }

        hood.tuningUpdate(HOOD_POSITION);

        robotCentricDrive.update();

        telemetry.addLine("<dpad up> to add data point.");
        telemetry.addLine("<dpad down> to remove data point.");
        telemetry.addLine("<Y> to see hood interpolation dataset.");

        if (seeHoodInterpolationDataset) {
            telemetry.addData("positions", positions);
            telemetry.addData("distances", distances);
            telemetry.addData("velocities", velocities);
        }
        else {
            telemetry.addLine();

            telemetry.addData("distance to goal", DISTANCE_TO_GOAL);
            telemetry.addData("hood position", hood.accessHoodAngler().getPosition());

            telemetry.addLine();

            telemetry.addData("loop ms", HardwareMonitor.get().getLastLoopMs());

            telemetry.addLine();

            telemetry.addData("flywheel current velocity", flywheel.getCurrentVelocity());
            telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

            telemetry.addData("p", flywheel.p);
            telemetry.addData("i", flywheel.i);
            telemetry.addData("d", flywheel.d);
            telemetry.addData("v", flywheel.v);
        }

        telemetry.update();

    }
}
