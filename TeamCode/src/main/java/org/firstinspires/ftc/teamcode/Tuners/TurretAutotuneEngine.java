package org.firstinspires.ftc.teamcode.Tuners;

import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

//no hardware: the OpMode feeds it the turret position (ticks from home) and sends out its volts, the sim does the same
public class TurretAutotuneEngine {

    public static final String[] KNOBS = {"kp_far", "kp_close", "ki_far", "ki_close", "kd_far", "kd_close", "k_hold_volts", "friction_deadband_degrees", "max_i", "stribeck_degrees_per_second",
            "pulse_over_breakaway_volts", "pulse_wait_seconds", "pulse_zone_degrees", "friction_ramp_degrees_per_second", "rate_feedforward_scale", "kinetic_trim_volts", "breakaway_trim_volts"};

    static final double[] MIN = {0.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0.08, 0, 0, 0.5, -0.4, -0.4};
    static final double[] MAX = {20, 20, 40, 60, 1.5, 1.5, 1.5, 3, 6000, 30, 0.6, 0.4, 15, 200, 1.3, 0.4, 0.4};
    static final double[] STEP = {1, 1, 3, 4, 0.1, 0.1, 0.1, 0.25, 400, 0.5, 0.05, 0.04, 2, 20, 0.1, 0.05, 0.05};
    //step moves tune these, then the match course tunes the hold and tracking ones
    static final int[] TUNE_KNOBS = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static final int[] MATCH_KNOBS = {1, 3, 5, 7, 10, 11, 12, 13, 14, 15, 16};
    public static final int MATCH_ROUNDS = 1;

    //trial moves, deg from home; each target is held SEGMENT s
    static final double[] TARGETS = {10, -10, 40, -40, 3, -3, 80, 0};
    static final double SEGMENT = 1.4, SCORE_WINDOW = 0.4, SAFE_DEGREES = 110;
    static final double RAMP_RATE = 2, RAMP_MAX = 7, MOVED_DEGREES = 0.4, RAMP_LAG = 0.12;
    static final double[] SPEED_OFFSETS = {0.2, 0.5, 0.9};
    static final double PREP_DEGREES = 60;
    static final double HOLD_DEGREES = 120, HOLD_SECONDS = 1.5, HOLD_SKIP = 0.25, COAST_GUESS = 900;
    //each kick that shows no count is retried longer; any real count (MOVED_DEGREES) passes
    static final double KICK_VOLTS = 4, KICK_LOOK = 0.4;
    static final double[] KICK_SECONDS = {0.15, 0.22, 0.3};
    public static final int ROUNDS = 2;

    public int rounds = ROUNDS;
    static final double KEEP_MARGIN = 0.95;

    enum Stage {
        REST, KICK, RAMP, SETTLE, SPEED, TUNE, MATCH, DONE, FAULT
    }

    private final double tpd;
    private final TurretController controller = new TurretController();
    private final double[] knobs, startKnobs, step = STEP.clone();
    private int[] search = TUNE_KNOBS;
    private boolean matching;
    private double rateScale = 1, tuneCost = Double.NaN;
    private double gain, kinetic, breakaway;

    private Stage stage = Stage.REST, after = Stage.KICK;
    private boolean fresh = true;
    private String status = "starting";
    private final List<String> history = new ArrayList<>();
    private final Map<String, Double> results = new LinkedHashMap<>();

    private double volts, target, stageStart = Double.NaN, lastTime = Double.NaN, lastPosition, stillSince;

    private int ramp;
    private final double[] breakaways = new double[4];
    private double rampStart;

    private int speedIndex;
    private final double[] speedVolts = new double[2 * SPEED_OFFSETS.length], speeds = new double[2 * SPEED_OFFSETS.length];
    private double holdStart, holdDir, sumT, sumX, sumTT, sumTX;
    private int holdN;

    private int segment, knob = -1, direction, round, trials;
    private double trialCost, baseline = Double.NaN, firstCost = Double.NaN, segStart, settledAt, overshoot, winN, winSum, winSumSq, winErr, flips, lastVolts, stalledSince = Double.NaN;
    private boolean stalled;

    public TurretAutotuneEngine(double ticksPerDegree, double[] startKnobs, double gainGuess, double kineticGuess, double breakawayGuess) {

        tpd = ticksPerDegree;
        this.startKnobs = withDefaults(startKnobs);
        knobs = this.startKnobs.clone();
        gain = gainGuess;
        kinetic = kineticGuess;
        breakaway = breakawayGuess;
    }

    //callers may pass only the step-move knobs: the rest start from TurretConstants
    static double[] withDefaults(double[] k) {

        double[] all = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, TurretConstants.TURRET_PULSE_OVER_BREAKAWAY_VOLTS, TurretConstants.TURRET_PULSE_WAIT_SECONDS, TurretConstants.TURRET_PULSE_ZONE_DEGREES, TurretConstants.TURRET_FRICTION_RAMP_DEGREES_PER_SECOND, TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE, 0, 0};
        System.arraycopy(k, 0, all, 0, Math.min(k.length, all.length));
        for (int i = 0; i < all.length; i++) all[i] = Math.max(MIN[i], Math.min(MAX[i], all[i]));
        return all;
    }

    public void update(double now, double position) {

        if (Double.isNaN(stageStart)) {
            stageStart = now;
            lastTime = now;
            lastPosition = position;
            stillSince = now;
        }
        double dt = now - lastTime;
        boolean moved = position != lastPosition;
        if (moved) stillSince = now;
        if (stage != Stage.DONE && stage != Stage.FAULT && Math.abs(position) > SAFE_DEGREES * tpd) {
            fault(String.format(Locale.US, "turret at %.0f deg, past %.0f: check the encoder (plugged in, coupled) and TURRET_BASE_DIRECTIONS / TURRET_ENCODER_DIRECTION", position / tpd, SAFE_DEGREES));
        }
        Stage was = stage;
        switch (stage) {
            case REST:
                volts = 0;
                status = "keep the robot still, hands clear";
                //the servos ignore commands for ~1.1 s after START
                if (now - stageStart > 0.5 && now >= TurretConstants.TURRET_SERVO_WAKE_SECONDS + 0.2) next(after, now);
                break;
            case KICK:
                kick(now, position);
                break;
            case RAMP:
                ramp(now, position);
                break;
            case SETTLE:
                volts = 0;
                if (now - stageStart > 0.4 && now - stillSince > 0.25) next(after, now);
                break;
            case SPEED:
                speed(now, position);
                break;
            case TUNE:
                tune(now, dt, position);
                break;
            case MATCH:
                match(now, dt, position);
                break;
            default:
                volts = 0;
                break;
        }
        if (stage == was) fresh = false;
        lastTime = now;
        lastPosition = position;
    }

    private void next(Stage s, double now) {

        stage = s;
        stageStart = now;
        fresh = true;
    }

    private void fault(String why) {

        stage = Stage.FAULT;
        volts = 0;
        status = why;
        history.add("FAULT: " + why);
    }

    //away from the side it's on, so it stays near home
    private double towardHome(double position) {
        return position > 0 ? -1 : 1;
    }

    private int kicks;
    private double kickStart, holdSpeed, holdLast, holdLastT;

    private int kickTry;
    private double kickDir = 1;

    //short push each way first: no counts after every try, or counts the wrong way, faults before anything long runs
    private void kick(double now, double position) {

        double dir = kickDir;
        if (fresh) kickStart = position;
        double t = now - stageStart, push = KICK_SECONDS[kickTry];
        volts = t < push ? dir * KICK_VOLTS : 0;
        status = "checking the encoder";
        if (t < push + KICK_LOOK) return;
        double moved = (position - kickStart) * dir;
        if (Math.abs(moved) < MOVED_DEGREES * tpd) {
            if (++kickTry < KICK_SECONDS.length) {
                //retries alternate, so a dead encoder can't carry it far one way
                kickDir = -kickDir;
                history.add(String.format(Locale.US, "%.0f V for %.2f s: no count, trying longer", KICK_VOLTS, push));
                after = Stage.KICK;
                next(Stage.SETTLE, now);
                return;
            }
            fault(String.format(Locale.US, "%.0f V kicks up to %.2f s and the encoder never counted (%.2f deg): encoder unplugged/uncoupled or turret jammed", KICK_VOLTS, push, Math.abs(moved) / tpd));
            return;
        }
        if (moved < 0) {
            fault("turret moved the wrong way for positive volts: flip TURRET_BASE_DIRECTIONS or TURRET_ENCODER_DIRECTION");
            return;
        }
        history.add(String.format(Locale.US, "encoder ok: %.0f V kick %s for %.2f s moved %.1f deg", KICK_VOLTS, dir > 0 ? "+" : "-", push, moved / tpd));
        kicks++;
        kickTry = 0;
        kickDir = -kickDir;
        after = kicks < 2 ? Stage.KICK : Stage.RAMP;
        next(Stage.SETTLE, now);
    }

    private void ramp(double now, double position) {

        double t = now - stageStart;
        double dir = ramp % 2 == 0 ? 1 : -1;
        if (fresh) rampStart = position;
        volts = dir * RAMP_RATE * t;
        status = String.format(Locale.US, "breakaway %d/4: %.2f V", ramp + 1, Math.abs(volts));
        double moved = (position - rampStart) * dir;
        if (moved < -MOVED_DEGREES * tpd) {
            fault("turret moved the wrong way for positive volts: flip TURRET_BASE_DIRECTIONS or TURRET_ENCODER_DIRECTION");
            return;
        }
        if (moved > MOVED_DEGREES * tpd) {
            breakaways[ramp] = Math.max(0, Math.abs(volts) - RAMP_RATE * RAMP_LAG);
            history.add(String.format(Locale.US, "breakaway %s: %.2f V", dir > 0 ? "+" : "-", breakaways[ramp]));
            ramp++;
            volts = 0;
            if (ramp < breakaways.length) {
                after = Stage.RAMP;
                next(Stage.SETTLE, now);
            }
            else {
                breakaway = median(breakaways, breakaways.length);
                after = Stage.SPEED;
                next(Stage.SETTLE, now);
            }
            return;
        }
        if (Math.abs(volts) > RAMP_MAX) fault(String.format(Locale.US, "no motion at %.0f V: encoder unplugged/uncoupled or the turret is jammed", RAMP_MAX));
    }

    private boolean prepping;
    private double holdT0, prepStill;

    private void speed(double now, double position) {

        if (fresh) {
            holdDir = speedIndex % 2 == 0 ? 1 : -1;
            prepping = true;
            prepStill = Double.NaN;
            holdSpeed = 0;
            holdLast = position;
            holdLastT = now;
        }
        if (now > holdLastT) {
            holdSpeed += 0.3 * (Math.abs(position - holdLast) / (now - holdLastT) - holdSpeed);
            holdLast = position;
            holdLastT = now;
        }
        if (prepping) {
            //crawl to the far side so the hold has room to reach its speed
            double e = (-holdDir * PREP_DEGREES * tpd - position) / tpd;
            boolean there = Math.abs(e) < 6;
            volts = there ? 0 : Math.signum(e) * (0.95 * breakaway + Math.min(1, 0.03 * Math.abs(e)));
            status = String.format(Locale.US, "speed %d/%d: getting into place", speedIndex + 1, speedVolts.length);
            if (there && holdSpeed < 2 * tpd) {
                if (Double.isNaN(prepStill)) prepStill = now;
            }
            else prepStill = Double.NaN;
            if ((!Double.isNaN(prepStill) && now - prepStill > 0.25) || now - stageStart > 4) {
                prepping = false;
                holdT0 = now;
                holdStart = position;
                sumT = sumX = sumTT = sumTX = 0;
                holdN = 0;
            }
            return;
        }
        double t = now - holdT0;
        double u = holdDir * (breakaway + SPEED_OFFSETS[speedIndex / 2]);
        volts = u;
        status = String.format(Locale.US, "speed %d/%d: %.1f V", speedIndex + 1, speedVolts.length, Math.abs(u));
        if (t > HOLD_SKIP) {
            sumT += t;
            sumX += position;
            sumTT += t * t;
            sumTX += t * position;
            holdN++;
        }
        double coast = holdSpeed * 0.1 + holdSpeed * holdSpeed / (2 * COAST_GUESS * tpd);
        if (t > HOLD_SECONDS || Math.abs(position - holdStart) + coast > HOLD_DEGREES * tpd) {
            double d = holdN * sumTT - sumT * sumT;
            speedVolts[speedIndex] = Math.abs(u);
            speeds[speedIndex] = holdN > 3 && d > 0 ? Math.abs((holdN * sumTX - sumT * sumX) / d) : Double.NaN;
            history.add(String.format(Locale.US, "%.1f V: %.0f deg/s", Math.abs(u), speeds[speedIndex] / tpd));
            speedIndex++;
            volts = 0;
            if (speedIndex < speedVolts.length) {
                after = Stage.SPEED;
                next(Stage.SETTLE, now);
            }
            else {
                fitSpeed();
                if (stage == Stage.FAULT) return;
                startTuning();
                after = Stage.TUNE;
                next(Stage.SETTLE, now);
            }
        }
    }

    private void fitSpeed() {

        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int k = 0; k < speeds.length; k++) {
            if (!(speeds[k] > 0)) continue;
            n++;
            sx += speedVolts[k];
            sy += speeds[k];
            sxx += speedVolts[k] * speedVolts[k];
            sxy += speedVolts[k] * speeds[k];
        }
        double d = n * sxx - sx * sx;
        double g = n >= 3 && d > 0 ? (n * sxy - sx * sy) / d : Double.NaN;
        double kc = g > 0 ? (sx - sy / g) / n : Double.NaN;
        if (!(g > 0) || !(kc > 0) || kc > 1.3 * breakaway) {
            fault(String.format(Locale.US, "speed fit failed (%.0f ticks/s per V, kinetic %.2f V): turret too erratic, try again", g, kc));
            return;
        }
        gain = g;
        kinetic = Math.min(kc, breakaway);
        breakaway = Math.max(breakaway, kinetic);
        history.add(String.format(Locale.US, "model: %.0f deg/s per V, friction %.2f / %.2f V", gain / tpd, kinetic, breakaway));
    }

    private void startTuning() {

        apply(knobs);
        segment = 0;
        knob = -1;
        direction = 0;
        round = 0;
        trialCost = 0;
    }

    private void apply(double[] k) {

        controller.applyConstants(tpd);
        controller.gain = gain;
        controller.kCoulomb = kinetic;
        controller.kStatic = breakaway;
        controller.kpFar = k[0];
        controller.kpClose = k[1];
        controller.kiFar = k[2];
        controller.kiClose = k[3];
        controller.kdFar = k[4];
        controller.kdClose = k[5];
        controller.kHold = k[6];
        controller.frictionDeadband = k[7] * tpd;
        controller.minI = -k[8];
        controller.maxI = k[8];
        controller.stribeckVelocity = k[9] * tpd;
        controller.pulseOverBreakaway = k[10];
        controller.pulseWait = k[11];
        controller.pulseZone = k[12] * tpd;
        controller.frictionRamp = k[13] * tpd;
        rateScale = k[14];
        controller.kCoulomb = Math.max(0, kinetic + k[15]);
        controller.kStatic = Math.max(controller.kCoulomb, breakaway + k[16]);
        //trials must compare the knobs, not a friction guess that drifted
        controller.frictionTrimStep = 0;
        controller.frictionTrim = 0;
        controller.reset();
    }

    private void tune(double now, double dt, double position) {

        if (fresh) {
            segStart = now;
            beginSegment(position);
        }
        target = TARGETS[segment] * tpd;
        double out = controller.update(dt, target, 0, position);
        volts = Math.max(-RAMP_MAX, Math.min(RAMP_MAX, out));

        double e = (target - position) / tpd, since = now - segStart;
        double sign = Math.signum(target - segmentFrom);
        overshoot = Math.max(overshoot, -sign * e);
        if (Math.abs(e) > 1) settledAt = Double.NaN;
        else if (Double.isNaN(settledAt)) settledAt = since;
        if (since > SEGMENT - SCORE_WINDOW) {
            winN++;
            winSum += position / tpd;
            winSumSq += (position / tpd) * (position / tpd);
            winErr += Math.abs(e);
            if (lastVolts * volts < 0) flips++;
        }
        if (Math.abs(volts) > 1.1 * breakaway && position == lastPosition) {
            if (Double.isNaN(stalledSince)) stalledSince = now;
            else if (now - stalledSince > 0.25) stalled = true;
        }
        else stalledSince = Double.NaN;
        lastVolts = volts;
        status = String.format(Locale.US, "%s round %d/%d, move %d/%d", knob < 0 ? "baseline" : (KNOBS[knob] + (direction > 0 ? " up" : " down")), round + 1, rounds, segment + 1, TARGETS.length);

        if (since < SEGMENT) return;
        trialCost += segmentCost();
        segment++;
        segStart = now;
        if (segment < TARGETS.length) {
            beginSegment(position);
            return;
        }
        finishTrial(now);
    }

    //the match course, deg from home: {seconds, kind, from, to}. kind 0 settle (not scored), 1 hold, 2 linear, 3 quadratic, 4 cubic,
    //5 wobble (sine round "from", amplitude "to"), 6 slow drift with aim jitter; the holds include no move at all and 0.5-2 deg nudges
    static final double[][] COURSE = {
            {0.8, 0, 0, 0}, {1.5, 2, 0, 45}, {1.2, 1, 45, 45}, {1.5, 3, 45, 0}, {1.0, 1, 0, 0}, {1.2, 4, 0, -50}, {1.0, 1, -50, -50},
            {2.5, 5, -20, 15}, {1.0, 1, -20, -20}, {0.8, 1, -19, -19}, {0.8, 1, -19.5, -19.5}, {0.8, 1, -17.5, -17.5}, {1.5, 1, -17.5, -17.5},
            {3.0, 6, -17.5, 10}, {1.5, 1, 30, 30}, {2.5, 4, 30, 0}, {1.0, 1, 0, 0}
    };
    static final double HOLD_SETTLE = 0.35, SWING_DEGREES = 0.3;

    private double firstMatchCost = Double.NaN, mMoveErr, mMoveN, mHoldErr, mHoldN, mSwings, mHoldTime, mFlips, mStalls, mExt = Double.NaN, mDir, mLast = Double.NaN;
    private final java.util.Random jitter = new java.util.Random(7);

    private void match(double now, double dt, double position) {

        if (fresh) {
            segStart = now;
            segment = 0;
            mMoveErr = mMoveN = mHoldErr = mHoldN = mSwings = mHoldTime = mFlips = mStalls = 0;
            jitter.setSeed(7);
            stalledSince = Double.NaN;
            beginHold();
        }
        double[] c = COURSE[segment];
        double since = now - segStart, x = Math.min(1, since / c[0]), aim, rate;
        int kind = (int) c[1];
        switch (kind) {
            case 2:
                aim = c[2] + (c[3] - c[2]) * x;
                rate = (c[3] - c[2]) / c[0];
                break;
            case 3:
                aim = c[2] + (c[3] - c[2]) * x * x;
                rate = (c[3] - c[2]) * 2 * x / c[0];
                break;
            case 4:
                aim = c[2] + (c[3] - c[2]) * (3 * x * x - 2 * x * x * x);
                rate = (c[3] - c[2]) * (6 * x - 6 * x * x) / c[0];
                break;
            case 5:
                aim = c[2] + c[3] * Math.sin(2 * Math.PI * 0.7 * since);
                rate = c[3] * 2 * Math.PI * 0.7 * Math.cos(2 * Math.PI * 0.7 * since);
                break;
            case 6:
                aim = c[2] + c[3] * Math.sin(2 * Math.PI * 0.15 * since) + 0.15 * jitter.nextGaussian();
                rate = c[3] * 2 * Math.PI * 0.15 * Math.cos(2 * Math.PI * 0.15 * since);
                break;
            default:
                aim = c[2];
                rate = 0;
                break;
        }
        target = aim * tpd;
        double out = controller.update(dt, target, rateScale * rate * tpd, position);
        volts = Math.max(-RAMP_MAX, Math.min(RAMP_MAX, out));

        double e = Math.abs(target - position) / tpd;
        if (kind == 1 && since > HOLD_SETTLE) {
            mHoldErr += e;
            mHoldN++;
            mHoldTime += dt;
            if (lastVolts * volts < 0) mFlips++;
            if (!Double.isNaN(mLast) && position != mLast) {
                double d = Math.signum(position - mLast);
                if (d != mDir) {
                    if (!Double.isNaN(mExt) && Math.abs(mLast - mExt) / tpd > SWING_DEGREES) mSwings++;
                    mExt = mLast;
                    mDir = d;
                }
            }
            mLast = position;
        }
        else if (kind > 1) {
            mMoveErr += e;
            mMoveN++;
        }
        if (Math.abs(volts) > 1.1 * controller.kStatic + 0.3 && position == lastPosition) {
            if (Double.isNaN(stalledSince)) stalledSince = now;
            else if (now - stalledSince > 0.25) {
                mStalls++;
                stalledSince = now;
            }
        }
        else stalledSince = Double.NaN;
        lastVolts = volts;
        status = String.format(Locale.US, "match course: %s, part %d/%d", knob < 0 ? "baseline" : (KNOBS[search[knob]] + (direction > 0 ? " up" : " down")), segment + 1, COURSE.length);

        if (since < c[0]) return;
        segment++;
        segStart = now;
        beginHold();
        if (segment < COURSE.length) return;
        finishTrial(now);
    }

    private void beginHold() {

        mExt = mLast = Double.NaN;
        mDir = 0;
    }

    //lower is better; holding counts most, shake is what this is for
    private double matchCost() {

        double move = mMoveN > 0 ? mMoveErr / mMoveN : 5;
        double hold = mHoldN > 0 ? mHoldErr / mHoldN : 5;
        double swingRate = mHoldTime > 0 ? mSwings / mHoldTime : 0;
        double flipRate = mHoldTime > 0 ? mFlips / mHoldTime : 0;
        return move + 3 * hold + 1.5 * swingRate + 0.05 * flipRate + 3 * mStalls;
    }

    private double segmentFrom;

    private void beginSegment(double position) {

        segmentFrom = position;
        settledAt = Double.NaN;
        overshoot = 0;
        winN = winSum = winSumSq = winErr = flips = 0;
        stalled = false;
        stalledSince = Double.NaN;
    }

    //lower is better: settle time, overshoot past 0.5 deg, final error, holding jitter and volt flips (shake), stalls
    private double segmentCost() {

        double settle = Double.isNaN(settledAt) ? SEGMENT + 1 : settledAt;
        double mean = winN > 0 ? winSum / winN : 0;
        double jitter = winN > 1 ? Math.sqrt(Math.max(0, winSumSq / winN - mean * mean)) : 0;
        double finalError = winN > 0 ? winErr / winN : 5;
        double flipRate = flips / SCORE_WINDOW;
        return settle + 0.5 * Math.max(0, overshoot - 0.5) + 2 * finalError + 4 * jitter + 0.05 * flipRate + (stalled ? 3 : 0);
    }

    private void finishTrial(double now) {

        trials++;
        double cost = matching ? matchCost() : trialCost / TARGETS.length;
        trialCost = 0;
        segment = 0;
        if (knob < 0) {
            baseline = cost;
            if (matching && Double.isNaN(firstMatchCost)) firstMatchCost = cost;
            if (!matching && Double.isNaN(firstCost)) firstCost = cost;
            history.add(String.format(Locale.US, "%s round %d baseline: %.3f", matching ? "match course" : "steps", round + 1, cost));
            knob = 0;
            direction = 1;
        }
        else {
            int k = search[knob];
            if (cost < KEEP_MARGIN * baseline) {
                history.add(String.format(Locale.US, "%s %.4g -> %.4g: %.3f (kept)", KNOBS[k], knobs[k], trialValue(), cost));
                knobs[k] = trialValue();
                baseline = cost;
                step[k] *= 1.3;
                knob++;
                direction = 1;
            }
            else if (direction > 0) direction = -1;
            else {
                step[k] *= 0.5;
                knob++;
                direction = 1;
            }
            if (knob >= search.length && nextRound(now)) return;
        }
        while (knob >= 0 && trialValue() == knobs[search[knob]]) {
            if (direction > 0) direction = -1;
            else {
                knob++;
                direction = 1;
            }
            if (knob >= search.length && nextRound(now)) return;
        }
        double[] trial = knobs.clone();
        if (knob >= 0) trial[search[knob]] = trialValue();
        apply(trial);
        after = matching ? Stage.MATCH : Stage.TUNE;
        next(Stage.SETTLE, now);
    }

    //true when this phase is over: done, or the match course starts
    private boolean nextRound(double now) {

        round++;
        knob = -1;
        if (round < (matching ? MATCH_ROUNDS : rounds)) return false;
        if (matching) {
            finish();
            return true;
        }
        tuneCost = baseline;
        matching = true;
        search = MATCH_KNOBS;
        round = 0;
        apply(knobs);
        after = Stage.MATCH;
        next(Stage.SETTLE, now);
        return true;
    }

    private double trialValue() {

        int k = search[knob];
        return Math.max(MIN[k], Math.min(MAX[k], knobs[k] + direction * step[k]));
    }

    private void finish() {

        stage = Stage.DONE;
        volts = 0;
        double matchCost = matching ? baseline : Double.NaN;
        status = String.format(Locale.US, "done: steps %.3f -> %.3f, match course %.3f -> %.3f over %d trials", firstCost, matching ? tuneCost : baseline, firstMatchCost, matchCost, trials);
        history.add(status);
        double kc = Math.max(0, kinetic + knobs[15]);
        results.put("gain", gain);
        results.put("kinetic_friction", kc);
        results.put("breakaway_friction", Math.max(kc, breakaway + knobs[16]));
        for (int k = 0; k < KNOBS.length; k++) {
            if (k != 15 && k != 16) results.put(KNOBS[k], knobs[k]);
        }
        results.put("score_before", firstCost);
        results.put("score_after", matching ? tuneCost : baseline);
        results.put("match_score_before", firstMatchCost);
        results.put("match_score_after", matchCost);
    }

    //B once the step moves are done: keep what's tuned so far instead of throwing it all away
    public boolean keepEarly() {

        if (!matching || stage == Stage.DONE || stage == Stage.FAULT) return false;
        finish();
        return true;
    }

    static double median(double[] a, int n) {

        double[] b = new double[n];
        System.arraycopy(a, 0, b, 0, n);
        java.util.Arrays.sort(b);
        return n % 2 == 1 ? b[n / 2] : 0.5 * (b[n / 2 - 1] + b[n / 2]);
    }

    public double getVolts() {
        return volts;
    }

    public double getTarget() {
        return target;
    }

    public String getStage() {
        return stage.name();
    }

    public String getStatus() {
        return status;
    }

    public boolean isDone() {
        return stage == Stage.DONE;
    }

    public boolean isFaulted() {
        return stage == Stage.FAULT;
    }

    public List<String> getHistory() {
        return history;
    }

    public Map<String, Double> getResults() {
        return results;
    }

    public double[] getKnobs() {
        return knobs.clone();
    }

    public double[] getStartKnobs() {
        return startKnobs.clone();
    }

    public double getGain() {
        return gain;
    }

    public double getKinetic() {
        return kinetic;
    }

    public double getBreakaway() {
        return breakaway;
    }

    public TurretController getController() {
        return controller;
    }
}
