package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.plain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.CsvLogger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

//one OpMode, two modes: STATIONARY tunes from scratch, DRIVING retunes while you play
@Config
@TeleOp(name = "Turret Autotuner", group = "tuning")
public class TurretAutotuner extends TeleOpBaseOpMode {

    public static TurretAutotuneEngine.Mode MODE = TurretAutotuneEngine.Mode.STATIONARY;
    public static long PAD_LOOP_TO_MS = 0; //stationary loops are short, set this to TunedConstants.LOOP_TIME_MS to tune at match speed

    private static final String[] COLUMNS = {
            "t", "stage", "target", "target_vel", "reference", "position", "velocity", "left_volts", "right_volts",
            "available_volts", "heading", "omega", "p", "i", "d", "f", "v"
    };

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private RobotCentricDrive robotCentricDrive;

    private TurretAutotuneEngine engine;
    private CsvLogger logger;
    private final double[] row = new double[COLUMNS.length];
    private File folder;
    private String lastStage = "";
    private int stageNumber = 0;
    private boolean saved = false, flywheelOn = false;
    private double startTime;
    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeComponents(Alliance.BLUE);
        applyComponentTraits();

        DcMotor[] motors = {
                hardwareMap.dcMotor.get(DeviceConstants.leftFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightFrontMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.leftBackMotorDeviceName),
                hardwareMap.dcMotor.get(DeviceConstants.rightBackMotorDeviceName)
        };
        for (DcMotor motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        robotCentricDrive = new RobotCentricDrive(motors[0], motors[1], motors[2], motors[3], controller1);

        intake.provideComponents(new Intake(super.intake), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel);

        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        controller1.getInformation();
        if (controller1.dpad_upHasJustBeenPressed) MODE = TurretAutotuneEngine.Mode.STATIONARY;
        if (controller1.dpad_downHasJustBeenPressed) MODE = TurretAutotuneEngine.Mode.DRIVING;

        telemetry.addLine("TURRET AUTOTUNER");
        telemetry.addData("Mode (dpad up = stationary, dpad down = driving)", MODE);
        telemetry.addLine("Turret must point straight ahead when you hit INIT, that's 0.");
        if (MODE == TurretAutotuneEngine.Mode.STATIONARY) {
            telemetry.addLine("Robot stays still, on blocks or the floor. The turret is stopped past " + ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES + " / " + ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + " deg (ShooterConstants), keep hands and wires clear.");
            telemetry.addLine("Takes about 2 to 3 minutes and stops by itself.");
        }
        else {
            telemetry.addLine("Run STATIONARY first. After START, get ready and press A. Nothing else works until then.");
            telemetry.addLine("Then drive like a match (spin a lot). It finishes by itself, usually in about a minute.");
        }
        telemetry.update();
    }

    @Override
    public void start() {

        turret.setVoltageCompensation(TurretConstants.TURRET_VOLTAGE_COMPENSATION);
        TurretAutotuneEngine.Parameters current = new TurretAutotuneEngine.Parameters(TurretConstants.TURRET_GAIN, TurretConstants.TURRET_TIME_CONSTANT, TurretConstants.TURRET_KINETIC_FRICTION, TurretConstants.TURRET_BREAKAWAY_FRICTION, TurretConstants.TURRET_DEAD_TIME, TurretConstants.TURRET_BANDWIDTH, TurretConstants.TURRET_DAMPING, TurretConstants.TURRET_INTEGRAL_RATIO, TurretConstants.TURRET_FRICTION_SCALE).capability(TurretConstants.TURRET_MAX_SPEED, TurretConstants.TURRET_MAX_ACCELERATION).startDelay(TurretConstants.TURRET_START_DELAY).deceleration(TurretConstants.TURRET_MAX_DECELERATION);
        engine = new TurretAutotuneEngine(MODE, ShooterConstants.TURRET_TICKS_PER_DEGREE, ShooterConstants.TURRET_AIM_DIRECTION, current);

        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        folder = new File(new File(AppUtil.ROBOT_DATA_DIR, "TurretAutotune"), stamp + "-" + MODE.name().toLowerCase(Locale.US));
        logger = new CsvLogger(new File(folder, "samples.csv"), COLUMNS);
        startTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void loop() {

        long loopStart = System.nanoTime();
        clearCacheOfLynxModule();
        controller1.getInformation();
        controller2.getInformation();

        //decided before this loop's A press so starting doesn't also fire whatever A normally does
        boolean drivingAllowed = engine.isDrivingAllowed();
        if (engine.isWaitingForStart() && controller1.aHasJustBeenPressed) engine.requestStart();

        turret.readSensors();
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        double now = System.nanoTime() * 1e-9 - startTime;
        double availableVolts = turret.getCompensationVolts();

        engine.update(now, turret.getPosition() - turret.startPosition, turret.getVelocity(), availableVolts, pose.heading(), velocity.omega);
        turret.setVolts(engine.getLeftVolts(), engine.getRightVolts());

        follower.update();
        if (drivingAllowed) {
            robotCentricDrive.update();
            intake.update();
            blocker.update();
            if (controller1.left_bumperHasJustBeenPressed) flywheelOn = !flywheelOn;
        }
        else {
            flywheelOn = false;
            left_front.setPower(0);
            right_front.setPower(0);
            left_back.setPower(0);
            right_back.setPower(0);
        }
        if (flywheelOn) flywheel.setVelocity(ShooterConstants.DEFAULT_FLYWHEEL_VELOCITY, false);
        else flywheel.setVelocity(0, true);
        flywheel.update();

        if (!engine.getStage().equals(lastStage)) {
            lastStage = engine.getStage();
            stageNumber++;
        }
        log(now, availableVolts, pose, velocity);

        if ((engine.isDone() || engine.isFaulted()) && !saved) save();

        show();

        long left = PAD_LOOP_TO_MS - (System.nanoTime() - loopStart) / 1000000;
        if (left > 0) android.os.SystemClock.sleep(left);
    }

    @Override
    public void stop() {

        if (engine != null && !saved) save();
        if (logger != null) logger.close();
        turret.setVolts(0, 0);
    }

    private void log(double now, double availableVolts, Pose pose, Velocity velocity) {

        int k = 0;
        row[k++] = now;
        row[k++] = stageNumber;
        row[k++] = engine.getTarget();
        row[k++] = engine.getTargetVelocity();
        row[k++] = engine.getReference();
        row[k++] = turret.getPosition() - turret.startPosition;
        row[k++] = turret.getVelocity();
        row[k++] = engine.getLeftVolts();
        row[k++] = engine.getRightVolts();
        row[k++] = availableVolts;
        row[k++] = pose.heading();
        row[k++] = velocity.omega;
        row[k++] = engine.getController().p;
        row[k++] = engine.getController().i;
        row[k++] = engine.getController().d;
        row[k++] = engine.getController().f;
        row[k++] = engine.getController().v;
        logger.row(row);
    }

    private void show() {

        telemetry.addData("Mode", MODE);
        telemetry.addData("Stage", engine.getStage());
        telemetry.addLine(engine.getStatus());
        if (engine.isWaitingForStart()) telemetry.addLine(">>> PRESS A TO START <<<");
        telemetry.addData("Turret (deg)", "%.1f -> %.1f", (turret.getPosition() - turret.startPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE, engine.getTarget() / ShooterConstants.TURRET_TICKS_PER_DEGREE);

        for (String w : engine.getWarnings()) telemetry.addLine("! " + w);

        List<String> history = engine.getHistory();
        for (int k = Math.max(0, history.size() - 6); k < history.size(); k++) telemetry.addLine(history.get(k));

        if (engine.isDone()) {
            telemetry.addLine("DONE. Saved to " + folder.getPath());
            telemetry.addLine("Plug in and build, it goes into TunedConstants by itself. Or copy into TurretConstants:");
            Map<String, Double> r = engine.getResults();
            line("TURRET_GAIN", r.get("gain"));
            line("TURRET_TIME_CONSTANT", r.get("time_constant"));
            line("TURRET_DEAD_TIME", r.get("dead_time"));
            line("TURRET_KINETIC_FRICTION", r.get("kinetic_friction"));
            line("TURRET_BREAKAWAY_FRICTION", r.get("breakaway_friction"));
            line("TURRET_BANDWIDTH", r.get("bandwidth"));
            line("TURRET_DAMPING", r.get("damping"));
            line("TURRET_INTEGRAL_RATIO", r.get("integral_ratio"));
            line("TURRET_FRICTION_SCALE", r.get("friction_scale"));
            line("TURRET_MAX_SPEED", r.get("max_speed"));
            line("TURRET_MAX_ACCELERATION", r.get("max_acceleration"));
            line("TURRET_START_DELAY", r.get("start_delay"));
            line("TURRET_MAX_DECELERATION", r.get("max_deceleration"));
            line("TURRET_SHAKE_BANDWIDTH", r.get("shake_bandwidth"));
            telemetry.addLine("Results:");
            for (Map.Entry<String, Double> e : r.entrySet()) telemetry.addData(e.getKey(), e.getValue());
        }
        if (engine.isFaulted()) telemetry.addLine("STOPPED: " + engine.getStatus());
        telemetry.update();
    }

    private void line(String name, Double value) {
        telemetry.addData(name, value == null ? "-" : String.valueOf(value));
    }

    private void save() {

        saved = true;
        StringBuilder b = new StringBuilder("key,value\n");
        b.append("mode,").append(MODE.name().toLowerCase(Locale.US)).append('\n');
        b.append("status,").append(engine.isDone() ? "done" : engine.isFaulted() ? "fault" : "stopped").append('\n');
        b.append("message,").append(engine.getStatus().replace(',', ';')).append('\n');
        b.append("ticks_per_degree,").append(ShooterConstants.TURRET_TICKS_PER_DEGREE).append('\n');
        b.append("voltage_compensation,").append(TurretConstants.TURRET_VOLTAGE_COMPENSATION).append('\n');
        if (engine.isDone()) {
            for (Map.Entry<String, Double> e : engine.getResults().entrySet()) b.append(e.getKey()).append(',').append(e.getValue()).append('\n');
        }
        for (String w : engine.getWarnings()) b.append("warning,").append(w.replace(',', ';')).append('\n');
        for (String h : engine.getHistory()) b.append("history,").append(h.replace(',', ';')).append('\n');

        folder.mkdirs();
        File file = new File(folder, "result.csv");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
            w.write(b.toString());
        } catch (IOException e) {
            telemetry.log().add("couldn't write " + file);
        }
    }
}
