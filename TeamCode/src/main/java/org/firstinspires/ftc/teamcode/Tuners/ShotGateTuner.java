package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@TeleOp (group = "tuning")
public class ShotGateTuner extends TeleOpBaseOpMode {

    public static double TARGET_VELOCITY = ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY;
    //run the turret like TeleOp does (holding home), so the flywheel is tuned with its load and control running
    public static boolean RUN_TURRET = true;

    public static double VIBRATION_SECONDS = 3;

    //0 = auto
    public static double SHOT_DIP_THRESHOLD = 0;

    public static double ENTER_BAND_BELOW = FlywheelConstants.SHOT_GATE_ENTER_BAND_BELOW, ENTER_BAND_ABOVE = FlywheelConstants.SHOT_GATE_ENTER_BAND_ABOVE;
    public static double HOLD_BAND_BELOW = FlywheelConstants.SHOT_GATE_HOLD_BAND_BELOW, HOLD_BAND_ABOVE = FlywheelConstants.SHOT_GATE_HOLD_BAND_ABOVE;
    public static double DWELL = FlywheelConstants.SHOT_GATE_DWELL;
    public static double MIN_OPEN = FlywheelConstants.SHOT_GATE_MIN_OPEN;
    public static double PULSE = FlywheelConstants.SHOT_GATE_PULSE;

    private static final double DIP_WINDOW = 0.08;
    private static final double FEED_TAIL = 0.3;
    private static final double MIN_DIP_THRESHOLD = 40;
    private static final int MAX_VIBRATION_SAMPLES = 5000;

    private enum Phase { SPIN_UP, VIBRATION, SHOOTING, DONE }

    private static final class Stats {
        int shots;
        double errorSum, errorSquares, worstLow, worstHigh;
        double heldSeconds, waitingSeconds;
        int openingsWithShots, shotsInOpenings;
        int recoveringLoops, pSaturatedLoops;
        final List<Double> firstDipLatency = new ArrayList<>();
        final List<Double> ballSpacing = new ArrayList<>();
        final List<Double> recovery = new ArrayList<>();
    }

    private final Stats gateOff = new Stats();
    private final Stats gateOn = new Stats();

    private final Intake intake = new Intake().asSubsystem();
    private Drivetrain drivetrain;
    private Telemetry telemetry;

    private final ShotGate gate = new ShotGate(ENTER_BAND_BELOW, ENTER_BAND_ABOVE, HOLD_BAND_BELOW, HOLD_BAND_ABOVE, DWELL, MIN_OPEN, PULSE);
    private final ShotDipDetector detector = new ShotDipDetector(DIP_WINDOW, MIN_DIP_THRESHOLD);

    private Phase phase = Phase.SPIN_UP;
    private boolean useGate = false;

    private final double[] vibrationErrors = new double[MAX_VIBRATION_SAMPLES];
    private int vibrationCount = 0;
    private double vibrationStart, vibrationEnd;
    private double feedforwardVoltsSum = 0, batteryVoltsSum = 0;
    private double vibrationRms, vibrationPeakToPeak, vibrationHz, headroomVolts;

    private double lastLoopTime = Double.NaN, loopSecondsSum = 0;
    private int loopCount = 0;

    private boolean blockerOpen = false;
    private double openedAt = Double.NaN, closedAt = Double.NEGATIVE_INFINITY;
    private int shotsThisOpening = 0;
    private Stats openingStats = null;
    private double lastShotTime = Double.NaN;
    private boolean recovering = false;
    private double recoveryStart;
    private Stats recoveryStats;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeComponents(Alliance.BLUE);
        applyComponentTraits();

        drivetrain = createDrivetrain();
        intake.provideComponents(new Intake(super.intake), controller1);

        setUpLynxModule();

        telemetry.addLine("Load balls, press START and don't shoot until it says so.");
        telemetry.update();
    }

    @Override
    public void start() {

        flywheel.reset();
        blocker.setState(Blocker.BlockerState.BLOCK);
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();

        controller1.getInformation();
        controller2.getInformation();

        double now = System.nanoTime() * 1e-9;
        if (!Double.isNaN(lastLoopTime)) {
            loopSecondsSum += now - lastLoopTime;
            loopCount++;
        }
        double dt = Double.isNaN(lastLoopTime) ? 0 : now - lastLoopTime;
        lastLoopTime = now;

        flywheel.setVelocity(phase == Phase.DONE ? 0 : TARGET_VELOCITY, true);
        gate.configure(ENTER_BAND_BELOW, ENTER_BAND_ABOVE, HOLD_BAND_BELOW, HOLD_BAND_ABOVE, DWELL, MIN_OPEN, PULSE);

        double velocity = flywheel.getCurrentVelocity();
        double error = velocity - TARGET_VELOCITY;

        switch (phase) {
            case SPIN_UP:
                blocker.setState(Blocker.BlockerState.BLOCK);
                if (Math.abs(error) <= FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR) {
                    phase = Phase.VIBRATION;
                    vibrationStart = now;
                }
                break;
            case VIBRATION:
                blocker.setState(Blocker.BlockerState.BLOCK);
                recordVibration(now, error);
                if (now - vibrationStart >= VIBRATION_SECONDS || vibrationCount >= MAX_VIBRATION_SAMPLES) {
                    vibrationEnd = now;
                    analyzeVibration();
                    phase = Phase.SHOOTING;
                }
                break;
            case SHOOTING:
                shoot(now, dt, velocity, error);
                if (controller2.backHasJustBeenPressed) finish();
                break;
            case DONE:
                blocker.setState(Blocker.BlockerState.BLOCK);
                break;
        }

        follower.update();
        intake.update();
        flywheel.update();
        drivetrain.update();
        if (RUN_TURRET) {
            turret.setPosition(turret.startPosition);
            turret.setTargetVelocity(0);
            turret.update();
        }
        else turret.setVolts(0, 0);

        show(velocity, error);
    }

    private void recordVibration(double now, double error) {

        vibrationErrors[vibrationCount++] = error;
        feedforwardVoltsSum += flywheel.v + flywheel.s + flywheel.i;
        batteryVoltsSum += flywheel.getFilteredVolts();
    }

    private void analyzeVibration() {

        int n = Math.max(1, vibrationCount);

        double mean = 0;
        for (int k = 0; k < vibrationCount; k++) mean += vibrationErrors[k];
        mean /= n;

        double squares = 0, low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
        int crossings = 0;
        for (int k = 0; k < vibrationCount; k++) {
            double centered = vibrationErrors[k] - mean;
            squares += centered * centered;
            low = Math.min(low, vibrationErrors[k]);
            high = Math.max(high, vibrationErrors[k]);
            if (k > 0 && Math.signum(centered) != Math.signum(vibrationErrors[k - 1] - mean)) crossings++;
        }

        vibrationRms = Math.sqrt(squares / n);
        vibrationPeakToPeak = vibrationCount == 0 ? 0 : high - low;
        vibrationHz = crossings / (2 * Math.max(1e-6, vibrationEnd - vibrationStart));
        headroomVolts = batteryVoltsSum / n - feedforwardVoltsSum / n;

        detector.setThreshold(SHOT_DIP_THRESHOLD > 0 ? SHOT_DIP_THRESHOLD : Math.max(MIN_DIP_THRESHOLD, 1.5 * vibrationPeakToPeak));
    }

    private void shoot(double now, double dt, double velocity, double error) {

        if (controller2.aHasJustBeenPressed) useGate = !useGate;
        Stats stats = useGate ? gateOn : gateOff;

        boolean trigger = controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD);
        boolean ready = gate.update(now, TARGET_VELOCITY, velocity, blockerOpen);
        boolean open = trigger && (!useGate || ready);

        if (trigger) stats.heldSeconds += dt;
        if (trigger && !open) stats.waitingSeconds += dt;

        if (open && !blockerOpen) {
            finishOpening();
            openedAt = now;
            openingStats = stats;
            shotsThisOpening = 0;
            lastShotTime = Double.NaN;
        }
        if (!open && blockerOpen) closedAt = now;
        blockerOpen = open;
        blocker.setState(open ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);

        boolean feeding = open || now - closedAt < FEED_TAIL;

        if (detector.update(now, velocity, feeding)) {

            Stats owner = openingStats != null ? openingStats : stats;

            double contactError = detector.getContactVelocity() - TARGET_VELOCITY;
            owner.shots++;
            owner.errorSum += contactError;
            owner.errorSquares += contactError * contactError;
            owner.worstLow = Math.min(owner.worstLow, contactError);
            owner.worstHigh = Math.max(owner.worstHigh, contactError);

            if (shotsThisOpening == 0 && !Double.isNaN(openedAt)) owner.firstDipLatency.add(now - openedAt);
            if (!Double.isNaN(lastShotTime)) owner.ballSpacing.add(now - lastShotTime);
            shotsThisOpening++;
            lastShotTime = now;

            recovering = true;
            recoveryStart = now;
            recoveryStats = owner;
        }

        if (recovering) {
            recoveryStats.recoveringLoops++;
            if (flywheel.p >= flywheel.maxP - 1e-6) recoveryStats.pSaturatedLoops++;
            if (error >= -ENTER_BAND_BELOW && error <= ENTER_BAND_ABOVE) {
                recoveryStats.recovery.add(now - recoveryStart);
                recovering = false;
            }
        }
    }

    private void finishOpening() {

        if (openingStats != null && shotsThisOpening > 0) {
            openingStats.openingsWithShots++;
            openingStats.shotsInOpenings += shotsThisOpening;
        }
        openingStats = null;
        shotsThisOpening = 0;
    }

    private void finish() {

        finishOpening();
        phase = Phase.DONE;
        blocker.setState(Blocker.BlockerState.BLOCK);

        log("off", gateOff);
        log("on", gateOn);
        RobotLog.ii("ShotGateTuner", "vibrationRms=%.1f peakToPeak=%.1f hz=%.1f headroomV=%.2f dipThreshold=%.0f loopMs=%.1f", vibrationRms, vibrationPeakToPeak, vibrationHz, headroomVolts, detector.getThreshold(), meanLoopMs());
    }

    private void log(String name, Stats s) {

        if (s.shots == 0) return;
        RobotLog.ii("ShotGateTuner", "gate=%s shots=%d contactMean=%.1f contactStd=%.1f worstLow=%.1f worstHigh=%.1f rate=%.2f waitPct=%.1f ballsPerOpening=%.2f latency=%.3f spacing=%.3f recovery=%.3f pSatPct=%.0f", name, s.shots, mean(s), std(s), s.worstLow, s.worstHigh, rate(s), waitPct(s), ballsPerOpening(s), median(s.firstDipLatency), median(s.ballSpacing), median(s.recovery), pSaturatedPct(s));
    }

    private void show(double velocity, double error) {

        switch (phase) {
            case SPIN_UP:
                telemetry.addLine("Spinning up. Don't shoot yet.");
                break;
            case VIBRATION:
                telemetry.addLine("Measuring vibration. Don't shoot yet.");
                break;
            case SHOOTING:
                telemetry.addLine("Shoot like in a match (left trigger fires, right trigger feeds).");
                telemetry.addLine("Gamepad 2: A = gate on/off, BACK = finish. Do several bursts with each.");
                telemetry.addData("Gate", useGate ? "ON" : "OFF");
                telemetry.addData("Shots counted off / on", "%d / %d", gateOff.shots, gateOn.shots);
                break;
            case DONE:
                showResults();
                telemetry.update();
                return;
        }
        telemetry.addData("Flywheel speed", "%.0f", velocity);
        telemetry.addData("Error", "%.0f", error);
        telemetry.update();
    }

    private void showResults() {

        telemetry.addLine("Vibration (no shots):");
        telemetry.addData("RMS / peak to peak (ticks/s)", "%.1f / %.1f", vibrationRms, vibrationPeakToPeak);
        telemetry.addData("Frequency (Hz)", "%.1f", vibrationHz);
        telemetry.addData("Flywheel turns (rev/s)", "%.1f", TARGET_VELOCITY / FlywheelConstants.FLYWHEEL_TICKS_PER_REV);
        telemetry.addData("Dip threshold used (ticks/s)", "%.0f", detector.getThreshold());
        telemetry.addLine();

        telemetry.addLine("Speed when each ball touched the flywheel (ticks/s from target), OFF vs ON:");
        row("Shots counted", gateOff.shots, gateOn.shots, "%.0f");
        row("Average error", mean(gateOff), mean(gateOn), "%.1f");
        row("Spread (std)", std(gateOff), std(gateOn), "%.1f");
        row("Worst low", gateOff.worstLow, gateOn.worstLow, "%.0f");
        row("Worst high", gateOff.worstHigh, gateOn.worstHigh, "%.0f");
        row("Balls per second held", rate(gateOff), rate(gateOn), "%.2f");
        row("Time waiting on gate (%)", waitPct(gateOff), waitPct(gateOn), "%.0f");
        row("Balls per opening", ballsPerOpening(gateOff), ballsPerOpening(gateOn), "%.2f");
        row("Open to first dip (s)", median(gateOff.firstDipLatency), median(gateOn.firstDipLatency), "%.3f");
        row("Time between balls (s)", median(gateOff.ballSpacing), median(gateOn.ballSpacing), "%.3f");
        row("Recovery time (s)", median(gateOff.recovery), median(gateOn.recovery), "%.3f");
        row("P maxed during recovery (%)", pSaturatedPct(gateOff), pSaturatedPct(gateOn), "%.0f");
        telemetry.addLine();

        double band = Math.max(10, Math.round(2 * vibrationRms));
        double dwell = Math.max(0.02, Math.round(2 * meanLoopMs() / 5) * 0.005);
        double spacing = median(gateOn.ballSpacing);
        double pulse = ballsPerOpening(gateOn) > 1.3 && !Double.isNaN(spacing) ? Math.round(0.6 * spacing / 0.005) * 0.005 : 0;

        telemetry.addLine("Suggested FlywheelConstants:");
        telemetry.addData("SHOT_GATE_ENTER_BAND_BELOW / ABOVE", "%.0f", band);
        telemetry.addData("SHOT_GATE_HOLD_BAND_BELOW / ABOVE", "%.0f", 2 * band);
        telemetry.addData("SHOT_GATE_DWELL", "%.3f", dwell);
        telemetry.addData("SHOT_GATE_PULSE", "%.3f", pulse);

        if (pSaturatedPct(gateOn) > 50 || pSaturatedPct(gateOff) > 50) {
            telemetry.addLine();
            telemetry.addLine("P is maxed out while recovering, so recovery is slower than it needs to be.");
            telemetry.addData("Raise P_MAX in FlywheelTuner, up to about (V)", "%.1f", Math.max(flywheel.maxP, headroomVolts - 0.5));
        }
        if (vibrationRms > 0 && std(gateOn) > 2 * vibrationRms && gateOn.shots > 0) {
            telemetry.addLine();
            telemetry.addLine("The gate can't stop dips that happen after a ball is already released. Check the flywheel mechanically.");
        }
    }

    private void row(String name, double offValue, double onValue, String format) {
        telemetry.addData(name, "OFF " + format + "   ON " + format, offValue, onValue);
    }

    private double meanLoopMs() {
        return loopCount == 0 ? Double.NaN : 1000 * loopSecondsSum / loopCount;
    }

    private static double mean(Stats s) {
        return s.shots == 0 ? Double.NaN : s.errorSum / s.shots;
    }

    private static double std(Stats s) {

        if (s.shots == 0) return Double.NaN;
        double m = s.errorSum / s.shots;
        return Math.sqrt(Math.max(0, s.errorSquares / s.shots - m * m));
    }

    private static double rate(Stats s) {
        return s.heldSeconds <= 0 ? Double.NaN : s.shots / s.heldSeconds;
    }

    private static double waitPct(Stats s) {
        return s.heldSeconds <= 0 ? Double.NaN : 100 * s.waitingSeconds / s.heldSeconds;
    }

    private static double ballsPerOpening(Stats s) {
        return s.openingsWithShots == 0 ? Double.NaN : (double) s.shotsInOpenings / s.openingsWithShots;
    }

    private static double pSaturatedPct(Stats s) {
        return s.recoveringLoops == 0 ? Double.NaN : 100.0 * s.pSaturatedLoops / s.recoveringLoops;
    }

    private static double median(List<Double> values) {

        if (values.isEmpty()) return Double.NaN;
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted.get(sorted.size() / 2);
    }
}
