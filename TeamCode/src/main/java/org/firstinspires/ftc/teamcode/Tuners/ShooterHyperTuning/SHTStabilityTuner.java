package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.CsvLogger;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "SHT Stability Tuner", group = "tuning")
public class SHTStabilityTuner extends TeleOpBaseOpMode {

    public static final String FOLDER = "ShooterHyperTuning";

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Shooter shooter = new Shooter();
    private Drivetrain drivetrain;

    private FlywheelPIDVSCoefficients coefficients;
    private FlywheelHyperTuner flywheelTuner;
    private OnlineTuner.Knob[] allKnobs;
    private String resumedFrom = "defaults";

    private CsvLogger loopLog, eventLog, gainsLog;
    private double[] loopRow, gainsRow;
    private final double[] eventRow = new double[7];
    private final List<String> recent = new ArrayList<>();
    private File folder;
    private double startTime;

    private static final String[] LOOP_COLUMNS = {
            "t", "loop_ms", "fw_target", "fw_vel", "fw_power", "fw_boost", "feeding", "shots_seen",
            "tu_target_deg", "tu_pos_deg", "tu_vel_dps", "tu_volts", "tu_target_rate_dps",
            "omega_dps", "x", "y", "heading", "drive_holding",
            "fw_steady_cost", "fw_shot_cost"
    };

    @Override
    public void init() {

        initializeComponents(Alliance.RED);
        applyComponentTraits();
        drivetrain = createDrivetrain();

        FlywheelPIDVSCoefficients c = org.firstinspires.ftc.teamcode.Constants.FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        coefficients = new FlywheelPIDVSCoefficients(c.kpFar, c.kpClose, c.kiFar, c.kiClose, c.kd, c.kv, c.ks, c.ksSignWidth, c.iSwitch, c.pSwitch, c.kISmash, c.minP, c.maxP, c.minI, c.maxI, c.minD, c.maxD);
        flywheel.setVelocityPIDVSCoefficients(coefficients);
        flywheelTuner = new FlywheelHyperTuner(coefficients, flywheel.feedBoostVolts, flywheel.shotKickVolts, flywheel.shotKickTime);
        List<OnlineTuner.Knob> knobs = new ArrayList<>();
        for (OnlineTuner t : new OnlineTuner[] {flywheelTuner.stability, flywheelTuner.shots}) knobs.addAll(Arrays.asList(t.knobs));
        allKnobs = knobs.toArray(new OnlineTuner.Knob[0]);
        resume();
        flywheelTuner.apply(flywheel, coefficients);

        intake.provideComponents(new Intake(super.intake, fiArm), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel, turret);
        shooter.provideComponents(flywheel, turret, follower, controller1, controller2);
        addDriverTelemetry(shooter);
        record(shooter, blocker, intake, drivetrain);
        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        telemetry.addLine("SHT STABILITY TUNER (red): play it like a match.");
        telemetry.addLine("Shoot a lot, in bursts, from different spots.");
        telemetry.addLine("Flywheel gains change while you play, every change is logged. Starting from: " + resumedFrom);
        telemetry.update();
    }

    @Override
    public void start() {

        shooter.start(alliance);
        folder = new File(new File(AppUtil.ROBOT_DATA_DIR, FOLDER), new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()));
        String[] columns = new String[LOOP_COLUMNS.length + allKnobs.length];
        System.arraycopy(LOOP_COLUMNS, 0, columns, 0, LOOP_COLUMNS.length);
        for (int k = 0; k < allKnobs.length; k++) columns[LOOP_COLUMNS.length + k] = allKnobs[k].name;
        loopRow = new double[columns.length];
        loopLog = new CsvLogger(new File(folder, "loop.csv"), columns);
        eventLog = new CsvLogger(new File(folder, "events.csv"), "t", "tuner", "knob", "from", "to", "cost", "baseline_kept");
        String[] gainColumns = new String[allKnobs.length + 1];
        gainColumns[0] = "t";
        for (int k = 0; k < allKnobs.length; k++) gainColumns[k + 1] = allKnobs[k].name;
        gainsRow = new double[gainColumns.length];
        gainsLog = new CsvLogger(new File(folder, "gains.csv"), gainColumns);
        startTime = Clock.seconds();
        logGains(0);
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

        double now = Clock.seconds(), t = now - startTime;
        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE, dir = ShooterConstants.TURRET_AIM_DIRECTION;
        Velocity velocity = follower.velocity();
        double omega = Math.toDegrees(velocity.omega);
        double target = dir * turret.getTargetPosition() / tpd, position = dir * turret.getCurrentPosition() / tpd;
        double turretVelocity = dir * turret.getVelocity() / tpd, targetRate = dir * turret.getTargetVelocity() / tpd;
        double turretVolts = turret.getPower() * turret.getCompensationVolts() * dir;

        int stabilityTrials = flywheelTuner.stability.trials, shotTrials = flywheelTuner.shots.trials;
        flywheelTuner.update(now, flywheel.getTargetVelocity(), flywheel.getCurrentVelocity(), flywheel.isFeeding(), flywheel.shotsSeen);
        if (flywheelTuner.stability.trials > stabilityTrials) trial(t, 0, flywheelTuner.stability);
        if (flywheelTuner.shots.trials > shotTrials) trial(t, 1, flywheelTuner.shots);
        flywheelTuner.apply(flywheel, coefficients);

        log(t, target, position, turretVelocity, turretVolts, targetRate, omega);

        telemetry.addData("Recording", folder.getName());
        telemetry.addData("Flywheel stability", status(flywheelTuner.stability, flywheelTuner.lastSteadyCost));
        telemetry.addData("Flywheel shots", status(flywheelTuner.shots, flywheelTuner.lastShotCost));
        telemetry.addData("Drive hold", drivetrain.isHolding() ? "on (back: off)" : "off (back: on)");
        for (int k = Math.max(0, recent.size() - 4); k < recent.size(); k++) telemetry.addLine(recent.get(k));
        telemetry.addData("Loop ms", "%.1f", HardwareMonitor.get().getLastLoopMs());
        telemetry.update();
    }

    @Override
    public void stop() {

        super.stop();
        if (loopLog != null) loopLog.close();
        if (eventLog != null) eventLog.close();
        if (gainsLog != null) gainsLog.close();
        if (folder != null) {
            File file = new File(folder, "findings.md");
            try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
                w.write(Findings.markdown(folder.getName(), Clock.seconds() - startTime, flywheelTuner));
            } catch (IOException e) {
                telemetry.log().add("couldn't write " + file);
            }
        }
    }

    private void trial(double t, int tuner, OnlineTuner o) {

        OnlineTuner.Trial last = o.last;
        int knob = 0;
        for (int k = 0; k < allKnobs.length; k++) if (allKnobs[k].name.equals(last.knob)) knob = k;
        eventRow[0] = t;
        eventRow[1] = tuner;
        eventRow[2] = knob;
        eventRow[3] = last.from;
        eventRow[4] = last.to;
        eventRow[5] = last.cost;
        eventRow[6] = last.kept ? last.baseline : -last.baseline;
        eventLog.row(eventRow);
        recent.add(String.format(Locale.US, "%s %s %.4g -> %.4g: %.4g vs %.4g %s", o.name, last.knob, last.from, last.to, last.cost, last.baseline, last.kept ? "KEPT" : "no"));
        if (recent.size() > 20) recent.remove(0);
        if (last.kept) logGains(t);
    }

    private void logGains(double t) {

        gainsRow[0] = t;
        for (int k = 0; k < allKnobs.length; k++) gainsRow[k + 1] = allKnobs[k].value;
        gainsLog.row(gainsRow);
    }

    private void log(double t, double target, double position, double turretVelocity, double turretVolts, double targetRate, double omega) {

        Pose pose = follower.pose();
        int k = 0;
        loopRow[k++] = t;
        loopRow[k++] = HardwareMonitor.get().getLastLoopMs();
        loopRow[k++] = flywheel.getTargetVelocity();
        loopRow[k++] = flywheel.getCurrentVelocity();
        loopRow[k++] = flywheel.getPower();
        loopRow[k++] = flywheel.boost;
        loopRow[k++] = flywheel.isFeeding() ? 1 : 0;
        loopRow[k++] = flywheel.shotsSeen;
        loopRow[k++] = target;
        loopRow[k++] = position;
        loopRow[k++] = turretVelocity;
        loopRow[k++] = turretVolts;
        loopRow[k++] = targetRate;
        loopRow[k++] = omega;
        loopRow[k++] = pose.x();
        loopRow[k++] = pose.y();
        loopRow[k++] = pose.heading();
        loopRow[k++] = drivetrain.isHolding() ? 1 : 0;
        loopRow[k++] = flywheelTuner.lastSteadyCost;
        loopRow[k++] = flywheelTuner.lastShotCost;
        for (OnlineTuner.Knob knob : allKnobs) loopRow[k++] = knob.value;
        loopLog.row(loopRow);
    }

    private static String status(OnlineTuner o, double lastCost) {

        String trying = o.trying();
        return String.format(Locale.US, "cost %.4g (base %.4g), %s, %d/%d kept", lastCost, o.baseline, trying.isEmpty() ? "measuring" : "trying " + trying, o.keeps, o.trials);
    }

    private void resume() {

        File[] sessions = new File(AppUtil.ROBOT_DATA_DIR, FOLDER).listFiles();
        if (sessions == null) return;
        Arrays.sort(sessions);
        for (int s = sessions.length - 1; s >= 0; s--) {
            File gains = new File(sessions[s], "gains.csv");
            if (!gains.isFile()) continue;
            //no java.nio.file on the hub
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(gains), Charset.forName("UTF-8")))) {
                String header = reader.readLine(), line, lastLine = null;
                while ((line = reader.readLine()) != null) if (!line.trim().isEmpty()) lastLine = line;
                if (header == null || lastLine == null) continue;
                String[] names = header.split(","), values = lastLine.split(",");
                for (int k = 1; k < Math.min(names.length, values.length); k++) {
                    double v = Double.parseDouble(values[k]);
                    for (OnlineTuner.Knob knob : allKnobs) if (knob.name.equals(names[k].trim())) knob.value = OnlineTuner.clamp(v, knob.min, knob.max);
                }
                resumedFrom = sessions[s].getName();
                return;
            } catch (IOException | NumberFormatException e) {
                telemetry.log().add("couldn't read " + gains);
            }
        }
    }
}
