package org.firstinspires.ftc.teamcode.Tuners;

import org.firstinspires.ftc.teamcode.Systems.Drivetrain.DriveController;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/// Measures the drivetrain per axis (forward, strafe, turn) and tunes the pose hold, no hardware calls (the sims run it):
/// static friction (slow ramp), feedforward and lag (power steps both ways, so it comes back), traction (full power burst
/// then full reverse brake), then hold stiffness: it shoves itself (a motor pulse on top of the hold) at rising bandwidths
/// until the hold overshoots or chatters, and keeps a margin under the last good one.
/// Robot frame forward, left, ccw; field pose like Pedro. in, rad, s
public class DriveAutotuneEngine {

    public double travelLimit = 30, safetyRadius = 72, safetyHeading = Math.toRadians(45);
    public double rampRate = 0.2, stepTime = 0.8, burstTime = 0.3, stillTime = 0.3;
    public final double[] stepPowers = {0.3, 0.6};
    public double kickPower = 0.45, kickTime = 0.12, watchTime = 1.3;
    public double maxOvershoot = 0.15, maxChatter = 0.05, bandwidthMargin = 0.85;
    public final double[] bandwidthTrials = {4, 5, 6.25, 7.8, 9.8, 12.2, 15.3};

    private enum Stage { STATIC, STEP, BURST, BRAKE, HOLD, DONE, FAULT }

    private static final int SAMPLES = 256;

    private final DriveController controller = new DriveController();
    private final double[] power = new double[3], command = new double[3];
    private final Map<String, String> results = new LinkedHashMap<>();
    private final double[][] staticPower = new double[3][2], steadySpeed = new double[3][4];
    private final double[] tau = new double[3], good = new double[3];
    private final double[] sampleT = new double[SAMPLES], sampleV = new double[SAMPLES];
    private int samples;

    private Stage stage = Stage.STATIC;
    private int axis, dir = 1, step, trial, kickDir = 1;
    private double startX = Double.NaN, startY, axisX, axisY, axisH, stageStart, stillSince = Double.NaN;
    private double brakeFrom, burstSpeed, burstAt, lastPower, chatterSum, chatterN, peak, overshoot, smooth;
    private double lastT = Double.NaN, unwrapped, lastHeading;
    private boolean settling = true, newAxis = true;
    private String status = "starting", fault = "";

    public DriveAutotuneEngine() {

        for (int a = 0; a < 3; a++) {
            good[a] = Double.NaN;
            tau[a] = Double.NaN;
        }
    }

    public double[] getPower() {
        return power;
    }

    public boolean isDone() {
        return stage == Stage.DONE;
    }

    public boolean isFaulted() {
        return stage == Stage.FAULT;
    }

    public String getStatus() {
        return stage == Stage.FAULT ? "FAULT: " + fault : status;
    }

    public Map<String, String> getResults() {
        return results;
    }

    public DriveController getController() {
        return controller;
    }

    public void stop(String why) {

        stage = Stage.FAULT;
        fault = why;
        for (int a = 0; a < 3; a++) power[a] = 0;
    }

    /// pose and velocity field frame (Pedro), volts: battery
    public void update(double now, double x, double y, double heading, double vx, double vy, double omega, double volts) {

        for (int a = 0; a < 3; a++) power[a] = 0;
        if (stage == Stage.DONE || stage == Stage.FAULT) return;
        double dt = Double.isNaN(lastT) ? 0.02 : Math.max(now - lastT, 1e-3);
        lastT = now;
        if (Double.isNaN(startX)) {
            startX = x;
            startY = y;
            lastHeading = heading;
        }
        unwrapped += wrap(heading - lastHeading);
        lastHeading = heading;
        double fromStart = Math.hypot(x - startX, y - startY);
        if (fromStart > safetyRadius) {
            stop(String.format(Locale.US, "drove %.0f in from the start (limit %.0f): give it more room or lower travelLimit", fromStart, safetyRadius));
            return;
        }
        double ch = Math.cos(heading), sh = Math.sin(heading);
        double speed = axis == DriveController.FORWARD ? ch * vx + sh * vy : axis == DriveController.LEFT ? -sh * vx + ch * vy : omega;
        double moving = axis == DriveController.TURN ? 0.1 : 2;
        smooth += (speed - smooth) * Math.min(1, dt / 0.08);
        if (Math.abs(speed) < moving) {
            if (Double.isNaN(stillSince)) stillSince = now;
        }
        else stillSince = Double.NaN;
        if (settling) {
            if (!Double.isNaN(stillSince) && now - stillSince > stillTime) {
                settling = false;
                stageStart = now;
                samples = 0;
                peak = overshoot = chatterSum = chatterN = 0;
                axisX = x;
                axisY = y;
                axisH = axis == DriveController.TURN ? unwrapped : heading;
                if (stage == Stage.HOLD) controller.reset();
                newAxis = false;
            }
            return;
        }
        if (axis != DriveController.TURN && Math.abs(wrap(heading - axisH)) > safetyHeading && stage != Stage.HOLD) {
            stop(String.format(Locale.US, "turned %.0f deg while testing %s: check motor directions", Math.toDegrees(wrap(heading - axisH)), name(axis)));
            return;
        }
        double sideways = axis == DriveController.LEFT ? Math.PI / 2 : 0;
        double along = axis == DriveController.TURN ? unwrapped - axisH : Math.cos(axisH + sideways) * (x - axisX) + Math.sin(axisH + sideways) * (y - axisY);
        boolean far = axis != DriveController.TURN && Math.abs(along) > travelLimit;
        double elapsed = now - stageStart;
        switch (stage) {
            case STATIC: {
                double p = dir * rampRate * elapsed;
                status = String.format(Locale.US, "%s: static friction %s, %.2f power", name(axis), dir > 0 ? "+" : "-", Math.abs(p));
                if (Math.abs(speed) > moving) {
                    staticPower[axis][dir > 0 ? 0 : 1] = Math.abs(p);
                    if (dir > 0) next(Stage.STATIC, -1);
                    else next(Stage.STEP, 1);
                }
                else if (Math.abs(p) > 0.6) stop(name(axis) + " didn't move at 0.6 power: motors, directions or wiring");
                else power[axis] = p;
                break;
            }
            case STEP: {
                status = String.format(Locale.US, "%s: step %.2f %s", name(axis), stepPowers[step], dir > 0 ? "+" : "-");
                power[axis] = dir * stepPowers[step];
                if (samples < SAMPLES) {
                    sampleT[samples] = elapsed;
                    sampleV[samples++] = Math.abs(speed);
                }
                if (far || elapsed > stepTime) {
                    int k = 2 * step + (dir > 0 ? 0 : 1);
                    steadySpeed[axis][k] = Math.abs(smooth);
                    if (step == 0) tau[axis] = Double.isNaN(tau[axis]) ? riseTime(steadySpeed[axis][k]) : 0.5 * (tau[axis] + riseTime(steadySpeed[axis][k]));
                    if (dir > 0) next(Stage.STEP, -1);
                    else if (step + 1 < stepPowers.length) {
                        step++;
                        next(Stage.STEP, 1);
                    }
                    else next(Stage.BURST, 1);
                }
                break;
            }
            case BURST: {
                status = name(axis) + ": full power burst (traction)";
                power[axis] = dir;
                if (Double.isNaN(burstAt) && elapsed >= 0.15) {
                    burstAt = elapsed;
                    burstSpeed = Math.abs(speed);
                }
                if (elapsed > burstTime || far) {
                    stage = Stage.BRAKE;
                    stageStart = now;
                    brakeFrom = Math.abs(speed);
                }
                break;
            }
            case BRAKE: {
                status = name(axis) + ": full reverse brake";
                power[axis] = -dir;
                if (speed * dir <= 0 || elapsed > 1) {
                    record(axis, Double.isNaN(burstAt) ? Double.NaN : burstSpeed / burstAt, brakeFrom / Math.max(elapsed, 0.02));
                    trial = 0;
                    kickDir = 1;
                    next(Stage.HOLD, 1);
                }
                break;
            }
            case HOLD:
                hold(dt, x, y, heading, vx, vy, omega, volts, along, elapsed);
                break;
            default:
                break;
        }
    }

    private void hold(double dt, double x, double y, double heading, double vx, double vy, double omega, double volts, double along, double elapsed) {

        double bw = bandwidthTrials[trial];
        controller.bandwidth[axis] = bw;
        status = String.format(Locale.US, "%s: hold at %.1f rad/s, shove %s", name(axis), bw, kickDir > 0 ? "+" : "-");
        controller.update(dt, command, x, y, heading, vx, vy, omega, volts);
        for (int a = 0; a < 3; a++) power[a] = controller.power[a];
        //a shove: a motor pulse the hold has to fight off
        if (elapsed < kickTime) power[axis] = clamp(power[axis] + kickDir * kickPower, -1, 1);
        double offset = along * kickDir;
        if (elapsed >= kickTime) {
            peak = Math.max(peak, offset);
            overshoot = Math.max(overshoot, -offset);
        }
        if (elapsed > watchTime - 0.4) {
            double change = power[axis] - lastPower;
            chatterSum += change * change;
            chatterN++;
        }
        lastPower = power[axis];
        if (elapsed < watchTime) return;
        double chatter = chatterN > 0 ? Math.sqrt(chatterSum / chatterN) : 0;
        boolean ok = peak > 0 && overshoot <= maxOvershoot * peak && chatter <= maxChatter;
        results.put(name(axis) + "_trial_" + fmt(bw) + (kickDir > 0 ? "_plus" : "_minus"), String.format(Locale.US, "peak %.3f overshoot %.3f chatter %.3f %s", peak, overshoot, chatter, ok ? "ok" : "no"));
        if (!ok) finishAxis();
        else if (kickDir > 0) {
            kickDir = -1;
            settling = true;
        }
        else {
            good[axis] = bw;
            kickDir = 1;
            if (trial + 1 < bandwidthTrials.length) {
                trial++;
                settling = true;
            }
            else finishAxis();
        }
    }

    private void finishAxis() {

        double bw = (Double.isNaN(good[axis]) ? bandwidthTrials[0] : good[axis]) * bandwidthMargin;
        controller.bandwidth[axis] = bw;
        results.put(key(axis, "bandwidth"), fmt(bw));
        if (axis == DriveController.TURN) {
            stage = Stage.DONE;
            status = "done";
            return;
        }
        axis++;
        step = 0;
        newAxis = true;
        next(Stage.STATIC, 1);
    }

    private void next(Stage s, int direction) {

        stage = s;
        dir = direction;
        settling = true;
        burstAt = Double.NaN;
    }

    //63% of the steady speed, from the samples of the step that just ran
    private double riseTime(double steady) {

        for (int k = 0; k < samples; k++) if (sampleV[k] >= 0.632 * steady) return Math.max(sampleT[k], 0.02);
        return Double.NaN;
    }

    private void record(int a, double accel, double decel) {

        double s1 = 0.5 * (steadySpeed[a][0] + steadySpeed[a][1]), s2 = 0.5 * (steadySpeed[a][2] + steadySpeed[a][3]);
        double kv = (stepPowers[1] - stepPowers[0]) / Math.max(s2 - s1, 1e-6);
        double ks = clamp(stepPowers[0] - kv * s1, 0, 0.3);
        double ka = (Double.isNaN(tau[a]) ? 0.15 : tau[a]) * kv;
        controller.kS[a] = ks;
        controller.kV[a] = kv;
        controller.kA[a] = ka;
        controller.maxAccel[a] = Double.isNaN(accel) ? Double.POSITIVE_INFINITY : accel;
        controller.maxDecel[a] = decel;
        results.put(key(a, "ks"), fmt(ks));
        results.put(key(a, "static_ks"), fmt(0.5 * (staticPower[a][0] + staticPower[a][1])));
        results.put(key(a, "kv"), fmt(kv));
        results.put(key(a, "ka"), fmt(ka));
        results.put(key(a, "time_constant"), fmt(tau[a]));
        results.put(key(a, "max_accel"), fmt(accel));
        results.put(key(a, "max_decel"), fmt(decel));
        results.put(key(a, "max_speed"), fmt((1 - ks) / kv));
    }

    static String name(int a) {
        return a == DriveController.FORWARD ? "forward" : a == DriveController.LEFT ? "strafe" : "turn";
    }

    static String key(int a, String what) {
        return name(a) + "_" + what;
    }

    static String fmt(double v) {
        return String.format(Locale.US, "%.6g", v);
    }

    static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
