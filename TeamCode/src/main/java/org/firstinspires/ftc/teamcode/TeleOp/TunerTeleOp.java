package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Utility.CsvLogger;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredVoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Config
@TeleOp (group = "A_Match")
public class TunerTeleOp extends TeleOpBaseOpMode {

    public static Alliance ALLIANCE = Alliance.BLUE;

    public static final String[] COLUMNS = {
            "t", "loop_ms", "extra_read_ms",
            "fw_target", "fw_raw", "fw_vel", "fw_filter_hz", "fw_power", "fw_p", "fw_i", "fw_d", "fw_v", "fw_s", "fw_volts_filtered", "fw_hub_volts",
            "tu_target", "tu_pos", "tu_error", "tu_raw", "tu_vel", "tu_filter_hz", "tu_target_vel", "tu_power", "tu_p", "tu_i", "tu_d", "tu_f", "tu_v", "tu_ref", "tu_volts_comp", "tu_hub_volts", "tu_vib_deg", "tu_bw", "tu_on_target", "tu_lanyard_v",
            "x", "y", "heading", "vx", "vy", "omega",
            "shooter_on", "auto_aim", "blocker_clear", "gate_ready", "intake"
    };

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Shooter shooter = new Shooter();

    private Drivetrain drivetrain;

    private VoltageSensor flywheelHubVoltage, turretHubVoltage;

    private CsvLogger logger;
    private final double[] row = new double[COLUMNS.length];
    private File sessionFolder;
    private double startTime;

    @Override
    public void init() {

        initializeComponents(ALLIANCE);

        applyComponentTraits();

        drivetrain = createDrivetrain();

        intake.provideComponents(new Intake(super.intake, fiArm), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel, turret);
        shooter.provideComponents(flywheel, turret, follower, controller1, controller2);

        flywheelHubVoltage = new MonitoredVoltageSensor(flywheel.getVoltageSensor(), "tunerFlywheelHub", false);
        turretHubVoltage = turret.getVoltageSensor() == flywheel.getVoltageSensor() ? null : new MonitoredVoltageSensor(turret.getVoltageSensor(), "tunerTurretHub", false);

        record(shooter, blocker, intake, drivetrain);
        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        controller1.getInformation();

        if (controller1.xHasJustBeenPressed) ALLIANCE = Alliance.BLUE;
        if (controller1.bHasJustBeenPressed) ALLIANCE = Alliance.RED;
        alliance.setAlliance(ALLIANCE);

        telemetry.addLine("Tuner TeleOp: plays like the match TeleOp and records everything.");
        telemetry.addData("Alliance (X = blue, B = red)", ALLIANCE);
        telemetry.update();
    }

    @Override
    public void start() {

        shooter.start(alliance);

        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        sessionFolder = new File(new File(AppUtil.ROBOT_DATA_DIR, "TunerTeleOp"), stamp);
        logger = new CsvLogger(new File(sessionFolder, "loop.csv"), COLUMNS);
        writeMeta();

        startTime = System.nanoTime() * 1e-9;
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
        drivetrain.update();

        log();

        telemetry.addData("Recording", sessionFolder.getName());
        telemetry.addData("Rows", logger.getRows());
        telemetry.addData("Loop ms", "%.1f", HardwareMonitor.get().getLastLoopMs());
        telemetry.update();
    }

    @Override
    public void stop() {

        super.stop();
        if (logger != null) logger.close();
    }

    private void log() {

        double readStart = System.nanoTime() * 1e-9;
        double fwHubVolts = flywheelHubVoltage.getVoltage();
        double tuHubVolts = turretHubVoltage == null ? fwHubVolts : turretHubVoltage.getVoltage();
        double now = System.nanoTime() * 1e-9;

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        Intake.IntakeState intakeState = intake.getState();

        int k = 0;
        row[k++] = now - startTime;
        row[k++] = HardwareMonitor.get().getLastLoopMs();
        row[k++] = 1000 * (now - readStart);
        row[k++] = flywheel.getTargetVelocity();
        row[k++] = flywheel.getRawVelocity();
        row[k++] = flywheel.getCurrentVelocity();
        row[k++] = flywheel.getEncoder().getFilterBandwidth();
        row[k++] = flywheel.getPower();
        row[k++] = flywheel.p;
        row[k++] = flywheel.i;
        row[k++] = flywheel.d;
        row[k++] = flywheel.v;
        row[k++] = flywheel.s;
        row[k++] = flywheel.getFilteredVolts();
        row[k++] = fwHubVolts;
        row[k++] = turret.getTargetPosition();
        row[k++] = turret.getCurrentPosition();
        row[k++] = turret.getError();
        row[k++] = turret.getRawVelocity();
        row[k++] = turret.getVelocity();
        row[k++] = turret.getFilterBandwidth();
        row[k++] = turret.getTargetVelocity();
        row[k++] = turret.getPower();
        row[k++] = turret.p;
        row[k++] = turret.i;
        row[k++] = turret.d;
        row[k++] = turret.f;
        row[k++] = turret.v;
        row[k++] = turret.getTargetPosition();
        row[k++] = turret.getCompensationVolts();
        row[k++] = tuHubVolts;
        row[k++] = Double.NaN;
        row[k++] = Double.NaN;
        row[k++] = turret.isOnTarget() ? 1 : 0;
        row[k++] = Double.NaN;
        row[k++] = pose.x();
        row[k++] = pose.y();
        row[k++] = pose.heading();
        row[k++] = velocity.vx;
        row[k++] = velocity.vy;
        row[k++] = velocity.omega;
        row[k++] = shooter.isShooterOn() ? 1 : 0;
        row[k++] = shooter.isAutoAimOn() ? 1 : 0;
        row[k++] = blocker.getState() == Blocker.BlockerState.CLEAR ? 1 : 0;
        row[k++] = blocker.isFlywheelReady() ? 1 : 0;
        row[k++] = intakeState == Intake.IntakeState.INTAKE ? 1 : intakeState == Intake.IntakeState.REVERSE_INTAKE ? -1 : 0;
        logger.row(row);
    }

    private void writeMeta() {

        StringBuilder meta = new StringBuilder("key,value\n");
        meta.append("alliance,").append(ALLIANCE).append('\n');
        meta.append("tuned_sessions,").append(TunedConstants.SESSIONS.replace(',', ' ')).append('\n');
        meta.append("flywheel_voltage_sample_period,").append(FlywheelConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD).append('\n');
        meta.append("flywheel_voltage_filter_tau,").append(FlywheelConstants.FLYWHEEL_VOLTAGE_FILTER_TAU).append('\n');
        meta.append("turret_voltage_sample_period,").append(TurretConstants.TURRET_VOLTAGE_SAMPLE_PERIOD).append('\n');
        meta.append("turret_voltage_filter_tau,").append(TurretConstants.TURRET_VOLTAGE_FILTER_TAU).append('\n');
        meta.append("turret_voltage_compensation,").append(TurretConstants.TURRET_VOLTAGE_COMPENSATION).append('\n');
        meta.append("turret_gain,").append(TurretConstants.TURRET_GAIN).append('\n');
        meta.append("turret_kinetic_friction,").append(TurretConstants.TURRET_KINETIC_FRICTION).append('\n');
        meta.append("turret_breakaway_friction,").append(TurretConstants.TURRET_BREAKAWAY_FRICTION).append('\n');
        meta.append("turret_autotune,").append(TunedConstants.TURRET_AUTOTUNE.replace(',', ' ')).append('\n');
        meta.append("turret_ticks_per_degree,").append(ShooterConstants.TURRET_TICKS_PER_DEGREE).append('\n');
        meta.append("shot_gate_enabled,").append(FlywheelConstants.SHOT_GATE_ENABLED).append('\n');
        meta.append("shot_gate_enter_band,").append(FlywheelConstants.SHOT_GATE_ENTER_BAND_BELOW).append('\n');
        meta.append("shot_gate_hold_band,").append(FlywheelConstants.SHOT_GATE_HOLD_BAND_BELOW).append('\n');
        meta.append("shot_gate_dwell,").append(FlywheelConstants.SHOT_GATE_DWELL).append('\n');
        meta.append("separate_turret_hub,").append(turretHubVoltage != null).append('\n');

        File file = new File(sessionFolder, "meta.csv");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
            w.write(meta.toString());
        } catch (IOException e) {
            telemetry.log().add("couldn't write " + file);
        }
    }
}
