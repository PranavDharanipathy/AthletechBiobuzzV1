package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
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
import org.firstinspires.ftc.teamcode.Utility.Recording.Recorder;
import org.firstinspires.ftc.teamcode.Utility.TemporalLog.TemporalLog;

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
    //every hardware read/write and every variable of this OpMode (shooter, flywheel, turret, blocker, intake, both tuners, pose) against time
    public static boolean TEMPORAL_LOG = true;
    public static double SYNC_MARK_SECONDS = 10;
    //sessions only resume from ones written the same way (knobs and costs changed on 2026-11-14: older gains aren't comparable)
    public static final String FORMAT = "2";
    //some bursts shoot off the table's speed, so the session's video shows where balls stop going in on each side: nothing else
    //says how wide the window is (docs/VIDEO_SHOT_REVIEW.md). One offset per burst, in turn; 0 = the table
    public static boolean PROBE = true;
    public static double[] PROBE_OFFSETS = {0, 0, -20, 0, 0, 12, 0, 0, -35};

    private final Intake intake = new Intake().asSubsystem();
    private final Blocker blocker = new Blocker().asSubsystem();
    private final Shooter shooter = new Shooter();
    private Drivetrain drivetrain;

    private FlywheelPIDVSCoefficients coefficients;
    private FlywheelHyperTuner flywheelTuner;
    private final TurretHyperTuner turretTuner = new TurretHyperTuner();
    private double lastLoopTime = Double.NaN;
    private int contactsSeen, starvedSeen, probeIndex;
    private double probeNext = Double.POSITIVE_INFINITY;
    private boolean inBurst;
    private OnlineTuner.Knob[] allKnobs;
    private String resumedFrom = "defaults";

    private CsvLogger loopLog, eventLog, gainsLog;
    private double[] loopRow, gainsRow;
    private final double[] eventRow = new double[7];
    private final List<String> recent = new ArrayList<>();
    private File folder;
    private double startTime, nextSyncMark = Double.NEGATIVE_INFINITY;
    private File temporalDir;

    private static final String[] LOOP_COLUMNS = {
            "t", "loop_ms", "fw_target", "fw_vel", "fw_power", "fw_boost", "feeding", "shots_seen",
            "tu_target_deg", "tu_pos_deg", "tu_vel_dps", "tu_volts", "tu_target_rate_dps",
            "omega_dps", "x", "y", "heading", "drive_holding",
            "fw_steady_cost", "fw_shot_cost",
            "fw_raw", "fw_observer", "fw_filtered", "fw_contacts", "fw_contact_err", "fw_ball_drop", "win_below", "win_above",
            "gate_open", "gate_predicted", "gate_holds", "gate_held_s", "battery", "table_probe"
    };

    @Override
    public void init() {

        //open before hardware so INIT's writes are in
        if (TEMPORAL_LOG) {
            temporalDir = TemporalLog.open("SHT Stability Tuner");
            TemporalLog.mark("phase.init", 0);
        }
        initializeComponents(Alliance.RED);
        applyComponentTraits();
        drivetrain = createDrivetrain();

        FlywheelPIDVSCoefficients c = org.firstinspires.ftc.teamcode.Constants.FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        coefficients = new FlywheelPIDVSCoefficients(c.kpFar, c.kpClose, c.kiFar, c.kiClose, c.kd, c.kv, c.ks, c.ksSignWidth, c.iSwitch, c.pSwitch, c.kISmash, c.minP, c.maxP, c.minI, c.maxI, c.minD, c.maxD);
        flywheel.setVelocityPIDVSCoefficients(coefficients);
        flywheelTuner = new FlywheelHyperTuner(coefficients, flywheel.observer.gain, flywheel.observer.lag, flywheel.observer.accelPerVolt, flywheel.observer.loadGain);
        List<OnlineTuner.Knob> knobs = new ArrayList<>();
        for (OnlineTuner t : new OnlineTuner[] {flywheelTuner.stability, flywheelTuner.shots, turretTuner.hold, turretTuner.track}) knobs.addAll(Arrays.asList(t.knobs));
        allKnobs = knobs.toArray(new OnlineTuner.Knob[0]);
        resume();
        flywheelTuner.apply(flywheel, coefficients);
        final double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        turret.overrides = controller -> turretTuner.apply(controller, tpd);
        shooter.turretRateScale = turretTuner.rateScale();

        intake.provideComponents(new Intake(super.intake, fiArm), controller1);
        blocker.provideComponents(super.blocker, controller1, flywheel, turret);
        //holds a ball the wheel won't be ready for: its holds and predictions are what decide whether the match TeleOps get it
        blocker.feedGateOn = true;
        shooter.provideComponents(flywheel, turret, follower, controller1, controller2);
        addDriverTelemetry(shooter);
        recorder.channel("sht", "probe", "ticks/s", "added to the table's speed this burst", () -> shooter.tableOffset);
        record(shooter, blocker, intake, drivetrain);
        setUpLynxModule();
        if (TEMPORAL_LOG) {
            TemporalLog.watch(this, "sht", FieldConstants.class, GeneralConstants.class, TunedConstants.class, FlywheelConstants.class, ShooterConstants.class, TurretConstants.class, BlockerConstants.class, IntakeConstants.class, DrivetrainConstants.class);
            TemporalLog.mark("phase.init_done", TemporalLog.variableCount());
            telemetry.addData("Temporal log", () -> temporalDir == null ? "FAILED to open (see logcat)" : temporalDir.getPath() + ", " + TemporalLog.records + " records, " + TemporalLog.dropped + " dropped");
        }
    }

    @Override
    public void init_loop() {

        TemporalLog.beginLoop();
        TemporalLog.endLoop();
        telemetry.addLine("SHT STABILITY TUNER (red): play it like a match.");
        telemetry.addLine("Shoot a lot, in bursts, from different spots.");
        telemetry.addLine("Flywheel and turret gains change while you play, every change is logged. Starting from: " + resumedFrom);
        telemetry.addLine("Hold still on targets sometimes and turn while aiming sometimes: the turret is scored on both.");
        telemetry.update();
    }

    @Override
    public void start() {

        TemporalLog.mark("phase.start", 0);
        shooter.start(alliance);
        folder = new File(new File(AppUtil.ROBOT_DATA_DIR, FOLDER), new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()));
        String[] columns = new String[LOOP_COLUMNS.length + allKnobs.length];
        System.arraycopy(LOOP_COLUMNS, 0, columns, 0, LOOP_COLUMNS.length);
        for (int k = 0; k < allKnobs.length; k++) columns[LOOP_COLUMNS.length + k] = allKnobs[k].name;
        loopRow = new double[columns.length];
        loopLog = new CsvLogger(new File(folder, "loop.csv"), columns);
        eventLog = new CsvLogger(new File(folder, "events.csv"), "t", "tuner", "knob", "from", "to", "cost", "baseline_kept");
        writeFormat();
        String[] gainColumns = new String[allKnobs.length + 1];
        gainColumns[0] = "t";
        for (int k = 0; k < allKnobs.length; k++) gainColumns[k + 1] = allKnobs[k].name;
        gainsRow = new double[gainColumns.length];
        gainsLog = new CsvLogger(new File(folder, "gains.csv"), gainColumns);
        startTime = Clock.seconds();
        logGains(0);
    }

    @Override
    protected boolean usesBlaze() {
        return true;
    }

    @Override
    protected void driveUpdate() {

        drivetrain.driveFromSticks();
        follower.update();
    }

    @Override
    protected void mechanismUpdate() {

        flywheel.update();
        turret.update();
        super.intake.update();
    }

    @Override
    public void loop() {

        synchronized (lock) {
            step();
        }
    }

    private void step() {

        TemporalLog.beginLoop();
        clearCacheOfLynxModule();
        //same instant in both logs (the robot log only starts at its first sample, just above): lines the temporal log up with the robot log and so with a video
        double clock = Clock.seconds();
        if (clock >= nextSyncMark) {
            TemporalLog.mark("sync", clock);
            Recorder.note("sync", "temporal log", clock);
            nextSyncMark = clock + SYNC_MARK_SECONDS;
        }
        controller1.getInformation();
        controller2.getInformation();

        shooter.mechanismsElsewhere = intake.motorElsewhere = !mechanismsInLoop();
        probe(Clock.seconds());
        blocker.update();
        updateFollower();
        intake.update();
        shooter.update();
        runDrive(drivetrain);

        double now = Clock.seconds(), t = now - startTime;
        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE, dir = ShooterConstants.TURRET_AIM_DIRECTION;
        Velocity velocity = follower.velocity();
        double omega = Math.toDegrees(velocity.omega);
        double target = dir * turret.getTargetPosition() / tpd, position = dir * turret.getCurrentPosition() / tpd;
        double turretVelocity = dir * turret.getVelocity() / tpd, targetRate = dir * turret.getTargetVelocity() / tpd;
        double turretVolts = turret.getPower() * turret.getCompensationVolts() * dir;

        int stabilityTrials = flywheelTuner.stability.trials, shotTrials = flywheelTuner.shots.trials;
        flywheelTuner.held(blocker.feedGate.heldSeconds);
        flywheelTuner.update(now, flywheel.getTargetVelocity(), flywheel.getCurrentVelocity(), flywheel.getRawVelocity(), flywheel.isFeeding());
        if (flywheel.getContacts() != contactsSeen) {
            contactsSeen = flywheel.getContacts();
            flywheelTuner.contact(flywheel.lastContactError, flywheel.windowBelow, flywheel.windowAbove);
        }
        if (blocker.feedGate.starved != starvedSeen) {
            starvedSeen = blocker.feedGate.starved;
            flywheelTuner.starved();
        }
        if (flywheelTuner.stability.trials > stabilityTrials) trial(t, 0, flywheelTuner.stability);
        if (flywheelTuner.shots.trials > shotTrials) trial(t, 1, flywheelTuner.shots);
        flywheelTuner.apply(flywheel, coefficients);

        double dt = lastLoopTime == lastLoopTime ? now - lastLoopTime : 0;
        lastLoopTime = now;
        int holdTrials = turretTuner.hold.trials, trackTrials = turretTuner.track.trials;
        turretTuner.update(now, dt, target, position, dir * shooter.turretAimVelocity / tpd, turretVolts, flywheel.isFeeding());
        if (turretTuner.hold.trials > holdTrials) trial(t, 2, turretTuner.hold);
        if (turretTuner.track.trials > trackTrials) trial(t, 3, turretTuner.track);
        shooter.turretRateScale = turretTuner.rateScale();

        log(t, target, position, turretVelocity, turretVolts, targetRate, omega);

        telemetry.addData("Recording", folder.getName());
        telemetry.addData("Flywheel stability", status(flywheelTuner.stability, flywheelTuner.lastSteadyCost));
        telemetry.addData("Flywheel shots", status(flywheelTuner.shots, flywheelTuner.lastShotCost));
        telemetry.addData("Balls", "%d seen hitting, %d outside their window (%.0f%%)", flywheelTuner.contacts, flywheelTuner.outside, flywheelTuner.contacts > 0 ? 100.0 * flywheelTuner.outside / flywheelTuner.contacts : 0.0);
        telemetry.addData("Last ball", "%+.0f ticks/s (window -%.0f/+%.0f), burst score %.2f, overshoot %.0f, gate held %.2f s", flywheelTuner.lastContactError, flywheel.windowBelow, flywheel.windowAbove, flywheelTuner.lastWindowScore, flywheelTuner.lastOvershoot, flywheelTuner.lastHeld);
        telemetry.addData("Feed gate", "%s, next ball %+.0f ticks/s, %d holds, %d presses with no ball out", blocker.feedGate.open ? "open" : "shut", blocker.feedGate.predictedError, blocker.feedGate.holds, blocker.feedGate.starved);
        if (PROBE) telemetry.addData("Probe", "%+.0f ticks/s off the table this burst", shooter.tableOffset);
        telemetry.addData("Ball expected in", "%.2f s (first %.2f s, then every %.2f s)", flywheel.expectedContact - Clock.seconds(), flywheel.firstContactSeconds, flywheel.ballIntervalSeconds);
        telemetry.addData("Turret hold", status(turretTuner.hold, turretTuner.lastHoldCost));
        telemetry.addData("Turret track", status(turretTuner.track, turretTuner.lastTrackCost));
        telemetry.addData("Drive hold", drivetrain.isHolding() ? "on (back: off)" : "off (back: on)");
        for (int k = Math.max(0, recent.size() - 4); k < recent.size(); k++) telemetry.addLine(recent.get(k));
        telemetry.addData("Loop ms", "%.1f", HardwareMonitor.get().getLastLoopMs());
        telemetry.update();
        TemporalLog.endLoop();
    }

    @Override
    public void stop() {

        TemporalLog.mark("phase.stop", 0);
        turret.overrides = null;
        shooter.turretRateScale = Double.NaN;
        super.stop();
        TemporalLog.close();
        if (loopLog != null) loopLog.close();
        if (eventLog != null) eventLog.close();
        if (gainsLog != null) gainsLog.close();
        if (folder != null) {
            File file = new File(folder, "findings.md");
            try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
                w.write(Findings.markdown(folder.getName(), Clock.seconds() - startTime, flywheelTuner, turretTuner));
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

    //the next offset once this burst's balls are all in (trigger let go, blocker shut, the last balls landed)
    private void probe(double now) {

        boolean burst = flywheel.isFeeding() || controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD);
        if (burst) probeNext = Double.POSITIVE_INFINITY;
        else if (inBurst) probeNext = now + FlywheelConstants.FLYWHEEL_CONTACT_AFTER_CLOSE;
        inBurst = burst;
        if (now >= probeNext) {
            probeIndex++;
            probeNext = Double.POSITIVE_INFINITY;
        }
        shooter.tableOffset = PROBE && PROBE_OFFSETS.length > 0 ? PROBE_OFFSETS[probeIndex % PROBE_OFFSETS.length] : 0;
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
        loopRow[k++] = flywheel.getRawVelocity();
        loopRow[k++] = flywheel.getObserverVelocity();
        loopRow[k++] = flywheel.getFilteredVelocity();
        loopRow[k++] = flywheel.getContacts();
        loopRow[k++] = flywheel.lastContactError;
        loopRow[k++] = flywheel.observer.ballDrop;
        loopRow[k++] = flywheel.windowBelow;
        loopRow[k++] = flywheel.windowAbove;
        loopRow[k++] = blocker.feedGate.open ? 1 : 0;
        loopRow[k++] = blocker.feedGate.predictedError;
        loopRow[k++] = blocker.feedGate.holds;
        loopRow[k++] = blocker.feedGate.heldSeconds;
        loopRow[k++] = flywheel.getFilteredVolts();
        loopRow[k++] = shooter.tableOffset;
        for (OnlineTuner.Knob knob : allKnobs) loopRow[k++] = knob.value;
        loopLog.row(loopRow);
    }

    private static String status(OnlineTuner o, double lastCost) {

        String trying = o.trying();
        return String.format(Locale.US, "cost %.4g (base %.4g), %s, %d/%d kept", lastCost, o.baseline, trying.isEmpty() ? "measuring" : "trying " + trying, o.keeps, o.trials);
    }

    private void writeFormat() {

        try (Writer w = new OutputStreamWriter(new FileOutputStream(new File(folder, "format.txt")), Charset.forName("UTF-8"))) {
            w.write(FORMAT);
        } catch (IOException e) {
            telemetry.log().add("couldn't write format.txt");
        }
    }

    private static String readFormat(File session) {

        File f = new File(session, "format.txt");
        if (!f.isFile()) return "";
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(f), Charset.forName("UTF-8")))) {
            String line = reader.readLine();
            return line == null ? "" : line.trim();
        } catch (IOException e) {
            return "";
        }
    }

    private void resume() {

        File[] sessions = new File(AppUtil.ROBOT_DATA_DIR, FOLDER).listFiles();
        if (sessions == null) return;
        Arrays.sort(sessions);
        for (int s = sessions.length - 1; s >= 0; s--) {
            File gains = new File(sessions[s], "gains.csv");
            if (!gains.isFile() || !FORMAT.equals(readFormat(sessions[s]))) continue;
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
