package org.firstinspires.ftc.teamcode.Tuners;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

//no hardware in here so it can run against a simulated turret, the OpMode feeds it sensors and sends out its volts
public class TurretAutotuneEngine {

    public enum Mode { STATIONARY, DRIVING }

    public static final double LOW_STEP_DEGREES = 60, HARD_STEP_DEGREES = 70; //travel of a speed step
    public static final double TRAVEL_INSET_DEGREES = 10; //the driving target stays this far inside the travel (what's left must still be a whole turn)
    public static final double SWING_MARGIN_DEGREES = 5; //spare room past the turret's stopping distance before it swings around

    public static final class Parameters {
        public double gain, timeConstant, kineticFriction, breakawayFriction, deadTime, bandwidth, damping, integralRatio, frictionScale;
        public double maxSpeed, maxAcceleration; //ticks/s, ticks/s^2, 0 = not measured
        public double startDelay; //s from rest, 0 = same as the dead time
        public double maxDeceleration; //ticks/s^2 coasting (the servos don't brake unless reversed), 0 = not measured
        public double shakeBandwidth; //rad/s vibration rejection may soften to while it shakes (DRIVING finds it), 0 = never soften

        public Parameters(double gain, double timeConstant, double kineticFriction, double breakawayFriction, double deadTime, double bandwidth, double damping, double integralRatio, double frictionScale) {

            this.gain = gain;
            this.timeConstant = timeConstant;
            this.kineticFriction = kineticFriction;
            this.breakawayFriction = breakawayFriction;
            this.deadTime = deadTime;
            this.bandwidth = bandwidth;
            this.damping = damping;
            this.integralRatio = integralRatio;
            this.frictionScale = frictionScale;
        }

        public Parameters capability(double maxSpeed, double maxAcceleration) {

            this.maxSpeed = maxSpeed;
            this.maxAcceleration = maxAcceleration;
            return this;
        }

        public Parameters deceleration(double maxDeceleration) {

            this.maxDeceleration = maxDeceleration;
            return this;
        }

        public Parameters startDelay(double startDelay) {

            this.startDelay = startDelay;
            return this;
        }

        public Parameters shakeBandwidth(double shakeBandwidth) {

            this.shakeBandwidth = shakeBandwidth;
            return this;
        }

        public Parameters copy() {
            return new Parameters(gain, timeConstant, kineticFriction, breakawayFriction, deadTime, bandwidth, damping, integralRatio, frictionScale).capability(maxSpeed, maxAcceleration).startDelay(startDelay).deceleration(maxDeceleration).shakeBandwidth(shakeBandwidth);
        }

        public void apply(TurretController c) {

            c.configure(gain, timeConstant, kineticFriction, breakawayFriction, deadTime, bandwidth, damping, integralRatio, frictionScale);
            c.setCapability(maxSpeed, maxAcceleration, maxDeceleration);
            c.startDelay = startDelay > 0 ? startDelay : Double.NaN;
        }
    }

    private interface Stage {
        String name();
        boolean run();
    }

    private final Mode mode;
    private final double tpd, aimDirection;
    private final TurretController controller = new TurretController();
    private final Deque<Stage> stages = new ArrayDeque<>();
    private Stage current;

    private Parameters params;

    private double t, dt, position, velocity, availableVolts, heading, omega;
    private double lastT = Double.NaN, loopAverage = Double.NaN;
    private double uLeft, uRight, directionSign = 1;
    private double target, targetVelocity;
    private boolean drivingAllowed, startRequested, done, faulted;
    private String status = "", fault = "";

    private final List<String> warnings = new ArrayList<>();
    private final List<String> history = new ArrayList<>();
    private final Map<String, Double> results = new LinkedHashMap<>();

    private double restNoise = Double.NaN, kickFriction = Double.NaN, breakaway = Double.NaN;
    private final List<double[]> stepPoints = new ArrayList<>(), plateauPoints = new ArrayList<>();
    private final List<double[]> lagPoints = new ArrayList<>();
    private final List<Double> stepDeadTimes = new ArrayList<>(), hardAccelerations = new ArrayList<>(), hardSpeeds = new ArrayList<>(), hardDeadTimes = new ArrayList<>();
    private double lowTopSpeed = 0, lowAcceleration = 0;
    private final List<Double> brakeDelays = new ArrayList<>(), coastDecelerations = new ArrayList<>();
    private boolean topSpeedReached = false;
    private double fitR2 = Double.NaN, asymmetry = Double.NaN;

    public TurretAutotuneEngine(Mode mode, double ticksPerDegree, double aimDirection, Parameters start) {

        this.mode = mode;
        this.tpd = ticksPerDegree;
        this.aimDirection = aimDirection;
        this.params = start.copy();
        controller.vibrationRejection = false; //tune the raw loop, rejection only helps on top of it
        if (mode == Mode.STATIONARY) controller.minHoldScale = 1; //and the hold push it learns

        if (mode == Mode.STATIONARY) {
            //nothing closed loop until the speed steps have measured the turret: a wrong guess runs it away
            stages.add(rest(0.6));
            stages.add(kick());
            stages.add(still());
            stages.add(singleServo(true));
            stages.add(still());
            stages.add(singleServo(false));
            stages.add(still());
            for (int k = 0; k < 4; k++) {
                stages.add(friction(k % 2 == 0 ? 1 : -1));
                stages.add(still());
            }
            stages.add(steps());
        }
        else {
            stages.add(waitForA());
            stages.add(drivingTune());
        }
    }

    public void requestStart() {
        startRequested = true;
    }

    public void update(double now, double positionTicks, double velocityTicks, double volts, double headingRad, double omegaRad) {

        dt = Double.isNaN(lastT) ? 0.01 : Math.max(1e-4, now - lastT);
        if (!Double.isNaN(lastT)) loopAverage = Double.isNaN(loopAverage) ? dt : loopAverage + 0.05 * (dt - loopAverage);
        lastT = now;
        t = now;
        position = positionTicks;
        velocity = velocityTicks;
        availableVolts = Math.max(1, 0.95 * volts);
        heading = headingRad;
        omega = omegaRad;

        if (faulted) {
            uLeft = uRight = 0;
            return;
        }

        if (Double.isNaN(position) || Double.isNaN(velocity)) {
            fail("encoder isn't reading (NaN)");
            return;
        }

        if (current == null) current = stages.poll();
        if (current == null) {
            if (!done) finish();
            hold(0);
        }
        else if (current.run()) {
            current = stages.poll();
        }

        double outward = Math.signum(position) * Math.signum(uLeft + uRight);
        double degrees = aimDirection * position / tpd;
        boolean past = degrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES || degrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES;
        if (past && outward > 0) fail("turret went past its travel (" + round(degrees, 1) + " deg, limits " + ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES + " / " + ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + " in ShooterConstants)");
    }

    public double getLeftVolts() {
        return faulted ? 0 : directionSign * uLeft;
    }

    public double getRightVolts() {
        return faulted ? 0 : directionSign * uRight;
    }

    public String getStage() {
        return faulted ? "FAULT" : done ? "DONE" : current == null ? "" : current.name();
    }

    public String getStatus() {
        return faulted ? fault : status;
    }

    public boolean isDone() {
        return done;
    }

    public boolean isFaulted() {
        return faulted;
    }

    public boolean isDrivingAllowed() {
        return drivingAllowed && !faulted;
    }

    public boolean isWaitingForStart() {
        return mode == Mode.DRIVING && !startRequested && !faulted;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getReference() {
        return controller.getReference();
    }

    public Parameters getParameters() {
        return params.copy();
    }

    public List<String> getHistory() {
        return history;
    }

    public List<String> getWarnings() {
        return warnings;
    }

    public Map<String, Double> getResults() {
        return results;
    }

    public TurretController getController() {
        return controller;
    }

    private void fail(String why) {

        faulted = true;
        fault = why;
        uLeft = uRight = 0;
    }

    private void out(double volts) {
        uLeft = uRight = clamp(volts, -availableVolts, availableVolts);
    }

    private void out(double left, double right) {

        uLeft = clamp(left, -availableVolts, availableVolts);
        uRight = clamp(right, -availableVolts, availableVolts);
    }

    private void hold(double targetTicks) {
        track(targetTicks, 0);
    }

    private void track(double targetTicks, double targetVelocityTicks) {

        target = targetTicks;
        targetVelocity = targetVelocityTicks;
        params.apply(controller);
        controller.setLimits(0.8, 3 * tpd, 1.5, 0.5 * tpd, tpd, 0.15 * tpd);
        controller.setTravel(aimDirection * ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * tpd, aimDirection * ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * tpd);
        out(controller.update(dt, targetTicks, targetVelocityTicks, position, velocity, availableVolts));
    }

    private double deg(double ticks) {
        return ticks / tpd;
    }


    private Stage rest(final double seconds) {

        return new Stage() {
            double start = Double.NaN, sum, squares;
            int n;

            public String name() {
                return "rest";
            }

            public boolean run() {

                if (Double.isNaN(start)) start = t;
                status = "measuring sensor noise, keep the robot still";
                out(0);
                sum += velocity;
                squares += velocity * velocity;
                n++;
                if (t - start < seconds) return false;
                double mean = sum / n;
                restNoise = Math.sqrt(Math.max(0, squares / n - mean * mean));
                return true;
            }
        };
    }

    private Stage kick() {

        return new Stage() {
            double start = Double.NaN, u, firstMotion = Double.NaN;

            public String name() {
                return "direction check";
            }

            public boolean run() {

                if (Double.isNaN(start)) start = t;
                status = "finding which way the turret turns";
                u = 3 * (t - start);
                if (Double.isNaN(firstMotion) && Math.abs(velocity) > 3 * tpd) firstMotion = u;
                if (Math.abs(position) > 2 * tpd) {
                    directionSign = Math.signum(position);
                    if (directionSign < 0) warnings.add("positive power turns the encoder negative: flip TURRET_ENCODER_DIRECTION (handled for this run)");
                    kickFriction = Double.isNaN(firstMotion) ? u : firstMotion;
                    out(0);
                    return true;
                }
                if (u > 9) {
                    fail("turret doesn't move at 9 V: servos fighting (check TURRET_BASE_DIRECTIONS), unplugged, jammed, or the encoder isn't counting");
                    return false;
                }
                out(u);
                return false;
            }
        };
    }

    /// volts off until the turret has stopped, so the next check doesn't see the last one's coasting
    private Stage still() {

        return new Stage() {
            double start = Double.NaN, calmSince = Double.NaN;

            public String name() {
                return "waiting for the turret to stop";
            }

            public boolean run() {

                if (Double.isNaN(start)) start = t;
                status = "waiting for the turret to stop";
                out(0);
                if (Math.abs(velocity) >= 2 * tpd) calmSince = Double.NaN;
                else if (Double.isNaN(calmSince)) calmSince = t;
                return !Double.isNaN(calmSince) && t - calmSince > 0.25 || t - start > 3;
            }
        };
    }

    private Stage moveTo(final double degrees, final String label) {

        return new Stage() {
            double start = Double.NaN, settledSince = Double.NaN;

            public String name() {
                return label;
            }

            public boolean run() {

                if (Double.isNaN(start)) {
                    start = t;
                    controller.reset(position);
                }
                status = label;
                Parameters saved = params;
                if (!identified) {
                    params = new Parameters(1800, 0.08, safe(kickFriction, 0.5) * 0.7, safe(kickFriction, 0.5), 0.05, 3, 1, 0.2, 1);
                }
                hold(degrees * tpd);
                params = saved;
                boolean close = Math.abs(position - degrees * tpd) < tpd && Math.abs(velocity) < 5 * tpd;
                if (close && Double.isNaN(settledSince)) settledSince = t;
                if (!close) settledSince = Double.NaN;
                if (!Double.isNaN(settledSince) && t - settledSince > 0.3) {
                    out(0);
                    return true;
                }
                if (t - start > 5) {
                    if (Math.abs(position - degrees * tpd) > 5 * tpd) fail("couldn't move the turret back to " + degrees + " degrees");
                    out(0);
                    return true;
                }
                return false;
            }
        };
    }

    private boolean identified = false;
    private double stretch = 1; //slow turrets get longer tests

    private Stage singleServo(final boolean left) {

        final String side = left ? "left" : "right";
        return new Stage() {
            double start = Double.NaN, startPosition;

            public String name() {
                return side + " servo check";
            }

            public boolean run() {

                if (Double.isNaN(start)) {
                    start = t;
                    startPosition = position;
                }
                status = "pushing with the " + side + " servo only";
                double u = 3 * (t - start), limit = Math.min(3 * safe(kickFriction, 1) + 2, 10);
                double moved = position - startPosition;
                if (Math.abs(moved) > tpd) {
                    out(0);
                    if (moved < 0 && directionSign < 0) fail("servos fight, the " + (left ? "right" : "left") + " one wins: flip the " + (left ? "right" : "left") + " entry in TURRET_BASE_DIRECTIONS only (or flip the " + side + " one and TURRET_ENCODER_DIRECTION)"); //the encoder warning came from the fight, not a real flip
                    else if (moved < 0) fail(side + " servo pushes the wrong way: flip its entry in TURRET_BASE_DIRECTIONS");
                    return true;
                }
                if (u > limit) {
                    warnings.add(side + " servo alone couldn't move the turret at " + round(limit, 1) + " V (dead, unplugged, or the other one holds it)");
                    out(0);
                    return true;
                }
                if (left) out(u, 0);
                else out(0, u);
                return false;
            }
        };
    }

    private final List<Double> breakaways = new ArrayList<>();

    private Stage friction(final double direction) {

        return new Stage() {
            double start = Double.NaN, startPosition;
            int moving;

            public String name() {
                return "friction";
            }

            public boolean run() {

                if (Double.isNaN(start)) {
                    start = t;
                    startPosition = position;
                }
                status = "measuring how hard it is to get the turret moving";
                double u = 2 * (t - start);
                boolean move = direction * velocity > 3 * tpd || direction * (position - startPosition) > 0.5 * tpd;
                moving = move ? moving + 1 : 0;
                if (moving >= 3) {
                    breakaways.add(Math.max(0, u - 2 * 2 * dt));
                    out(0);
                    return true;
                }
                if (u > 9) {
                    fail("turret didn't break free at 9 V");
                    return false;
                }
                out(direction * u);
                return false;
            }
        };
    }

    private Stage steps() {

        return new Stage() {
            boolean planned = false;

            public String name() {
                return "speed steps";
            }

            public boolean run() {

                if (!planned) {
                    planned = true;
                    breakaway = median(breakaways);
                    double minimum = Math.min(Collections.min(breakaways), Collections.max(breakaways));
                    if (Collections.max(breakaways) > 1.4 * Math.max(minimum, 0.05)) warnings.add("friction differs a lot between directions or spots (" + breakaways + " V)");
                    //small steps (open loop, back and forth so it stays near the middle) give speed per volt, they settle at a steady speed.
                    //Then full power steps give how fast it can speed up and its top speed: the servos run out of torque long before volts
                    double[] extra = {0.3, 0.6, 1.0, 1.6};
                    List<Stage> plan = new ArrayList<>();
                    for (double e : extra) {
                        for (int s = 0; s < 2; s++) {
                            plan.add(still());
                            plan.add(stepRun(s == 0 ? 1 : -1, breakaway + e, LOW_STEP_DEGREES, false));
                        }
                    }
                    plan.add(still());
                    plan.add(identify());
                    for (int s = 0; s < 2; s++) {
                        double direction = s == 0 ? 1 : -1;
                        plan.add(moveTo(-direction * HARD_STEP_DEGREES / 2, "lining up a full power step"));
                        plan.add(stepRun(direction, 9, HARD_STEP_DEGREES, true));
                    }
                    plan.add(moveTo(0, "centering"));
                    plan.add(capability());
                    for (int k = plan.size() - 1; k >= 0; k--) stages.addFirst(plan.get(k));
                }
                return true;
            }
        };
    }

    private Stage stepRun(final double direction, final double requested, final double travelDegrees, final boolean hard) {

        return new Stage() {
            double start = Double.NaN, startPosition, volts, brakeStart = Double.NaN;
            final List<double[]> samples = new ArrayList<>(), braking = new ArrayList<>();
            boolean coasted = false;

            public String name() {
                return "speed steps";
            }

            public boolean run() {

                if (Double.isNaN(start)) {
                    start = t;
                    startPosition = position;
                    volts = Math.min(requested, 0.85 * availableVolts);
                }
                status = "speed step " + round(volts, 1) + " V";
                if (Double.isNaN(brakeStart)) {
                    samples.add(new double[] {t - start, direction * (position - startPosition)});
                    if (direction * (position - startPosition) > travelDegrees * tpd || t - start > 1.2) {
                        brakeStart = t;
                        fit(samples, volts, hard);
                    }
                    else {
                        out(direction * volts);
                        return false;
                    }
                }
                braking.add(new double[] {t - brakeStart, direction * (position - startPosition)});
                if (!hard) {
                    //small steps coast to a stop with the command still forward under friction: that's how it slows in a match
                    double since = t - brakeStart;
                    if (!coasted && since < 0.35 && direction * velocity > 0.3 * Math.abs(samples.isEmpty() ? 0 : endSpeed(samples))) {
                        out(direction * 0.85 * breakaway);
                        return false;
                    }
                    if (!coasted) coastDeceleration(samples, braking);
                    coasted = true;
                    out(0);
                    if (direction * velocity < 3 * tpd || since > 1.2) {
                        out(0);
                        return true;
                    }
                    return false;
                }
                out(-direction * Math.min(9, availableVolts));
                if (direction * velocity < 3 * tpd || t - brakeStart > 0.4) {
                    out(0);
                    if (hard) brakeDelay(samples, braking);
                    return true;
                }
                return false;
            }
        };
    }

    private void fit(List<double[]> samples, double volts, boolean hard) {

        if (samples.size() < 8) return;
        speeds(samples, volts, hard);
        double bestSse = Double.POSITIVE_INFINITY, bestA = 0, bestTau = 0.05, bestTd = 0;
        for (double td = 0; td <= 0.15; td += 0.005) {
            for (int k = 0; k < 30; k++) {
                double tau = 0.01 * Math.pow(40, k / 29.0);
                double gx = 0, gg = 0;
                for (double[] s : samples) {
                    double g = basis(s[0], td, tau);
                    gx += g * s[1];
                    gg += g * g;
                }
                if (gg <= 0) continue;
                double a = gx / gg, sse = 0;
                for (double[] s : samples) {
                    double r = s[1] - a * basis(s[0], td, tau);
                    sse += r * r;
                }
                if (sse < bestSse) {
                    bestSse = sse;
                    bestA = a;
                    bestTau = tau;
                    bestTd = td;
                }
            }
        }
        if (bestA <= 0) return;
        stepDeadTimes.add(bestTd);
        if (hard) return; //its lag is the torque limit, not the servo's
        stepPoints.add(new double[] {volts, bestA, samples.get(samples.size() - 1)[1] > 0 ? 1 : 0});
        lagPoints.add(new double[] {volts, bestTau});
    }

    /// steady speed at the end (if it got there) and how fast it sped up, from the positions
    private void speeds(List<double[]> samples, double volts, boolean hard) {

        int n = samples.size();
        double end = samples.get(n - 1)[0];
        double late = slope(samples, 0.75 * end, end), before = slope(samples, 0.5 * end, 0.75 * end);
        if (!(late > 0)) return;
        boolean plateau = before > 0.9 * late;
        double accelerationNow = 0;
        double[] v = new double[n - 1], vt = new double[n - 1];
        for (int k = 0; k + 1 < n; k++) {
            double span = samples.get(k + 1)[0] - samples.get(k)[0];
            vt[k] = 0.5 * (samples.get(k + 1)[0] + samples.get(k)[0]);
            v[k] = span > 0 ? (samples.get(k + 1)[1] - samples.get(k)[1]) / span : 0;
        }
        int a = -1, b = -1;
        for (int k = 0; k < v.length; k++) {
            if (a < 0 && v[k] > 0.25 * late) a = k;
            if (a >= 0 && v[k] > 0.7 * late) {
                b = k;
                break;
            }
        }
        if (a >= 0 && b > a && vt[b] > vt[a]) accelerationNow = (v[b] - v[a]) / (vt[b] - vt[a]);
        if (hard) {
            hardSpeeds.add(late);
            if (plateau) topSpeedReached = true;
            if (accelerationNow > 0) {
                hardAccelerations.add(accelerationNow);
                hardDeadTimes.add(Math.max(0, vt[a] - v[a] / accelerationNow)); //the ramp is a straight line, where it starts is the dead time
            }
            return;
        }
        lowTopSpeed = Math.max(lowTopSpeed, late);
        lowAcceleration = Math.max(lowAcceleration, accelerationNow);
        if (plateau) plateauPoints.add(new double[] {volts, late});
    }

    private static double endSpeed(List<double[]> s) {

        if (s.size() < 2) return 0;
        double[] a = s.get(s.size() - 2), b = s.get(s.size() - 1);
        return (b[1] - a[1]) / Math.max(1e-4, b[0] - a[0]);
    }

    /// how fast it coasts down: the time from 85% to 40% of the speed it had (the delay before it starts slowing doesn't count)
    private void coastDeceleration(List<double[]> driving, List<double[]> coasting) {

        double from = endSpeed(driving);
        if (!(from > 60 * tpd) || coasting.size() < 3) return;
        double high = Double.NaN, low = Double.NaN, lastT = Double.NaN, lastV = from;
        for (int k = 1; k < coasting.size(); k++) {
            double[] a = coasting.get(k - 1), b = coasting.get(k);
            double span = b[0] - a[0];
            if (span <= 0) continue;
            double tm = 0.5 * (a[0] + b[0]), v = (b[1] - a[1]) / span;
            if (Double.isNaN(lastT)) lastT = tm - span;
            if (Double.isNaN(high) && v <= 0.85 * from) high = lastT + (tm - lastT) * (lastV - 0.85 * from) / Math.max(1e-9, lastV - v);
            if (Double.isNaN(low) && v <= 0.4 * from) {
                low = lastT + (tm - lastT) * (lastV - 0.4 * from) / Math.max(1e-9, lastV - v);
                break;
            }
            lastT = tm;
            lastV = v;
        }
        if (!Double.isNaN(high) && !Double.isNaN(low) && low > high) coastDecelerations.add(0.45 * from / (low - high));
    }

    /// how long a command takes to act once it's moving: the brake command until the speed drops (from rest it takes longer)
    private void brakeDelay(List<double[]> driving, List<double[]> braking) {

        if (driving.size() < 2 || braking.size() < 2) return;
        double[] a = driving.get(driving.size() - 2), b = driving.get(driving.size() - 1);
        double brakeAt = b[0];
        double speed = (b[1] - a[1]) / Math.max(1e-4, b[0] - a[0]);
        if (!(speed > 20 * tpd)) return;
        double[] previous = b;
        double lastFast = 0;
        for (double[] s : braking) {
            double at = brakeAt + s[0];
            double span = at - previous[0];
            if (span <= 0) continue;
            double v = (s[1] - previous[1]) / span;
            if (v >= 0.92 * speed) lastFast = 0.5 * (at + previous[0]) - brakeAt;
            else break;
            previous = new double[] {at, s[1]};
        }
        brakeDelays.add(Math.max(0, lastFast + 0.5 * safe(loopAverage, 0.02)));
    }

    private static double slope(List<double[]> samples, double from, double to) {

        double n = 0, st = 0, sx = 0, stt = 0, stx = 0;
        for (double[] s : samples) {
            if (s[0] < from || s[0] > to) continue;
            n++;
            st += s[0];
            sx += s[1];
            stt += s[0] * s[0];
            stx += s[0] * s[1];
        }
        double d = n * stt - st * st;
        return n >= 3 && d > 0 ? (n * stx - st * sx) / d : Double.NaN;
    }

    private static double basis(double time, double deadTime, double tau) {

        double s = time - deadTime;
        return s <= 0 ? 0 : s - tau * (1 - Math.exp(-s / tau));
    }

    private Stage identify() {

        return new Stage() {
            public String name() {
                return "calculating";
            }

            public boolean run() {

                double lowest = Double.POSITIVE_INFINITY, highest = 0;
                for (double[] p : plateauPoints) {
                    lowest = Math.min(lowest, p[0]);
                    highest = Math.max(highest, p[0]);
                }
                boolean steady = plateauPoints.size() >= 3 && highest - lowest > 0.25;
                List<double[]> points = steady ? plateauPoints : stepPoints;
                if (points.size() < 3) {
                    fail("speed steps didn't give usable data");
                    return false;
                }
                if (!steady) warnings.add("the small speed steps never settled to a steady speed, speed per volt is a guess");
                double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
                int n = points.size();
                for (double[] p : points) {
                    sx += p[0];
                    sy += p[1];
                    sxx += p[0] * p[0];
                    sxy += p[0] * p[1];
                    syy += p[1] * p[1];
                }
                double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
                double intercept = (sy - slope * sx) / n;
                double r = (n * sxy - sx * sy) / Math.sqrt((n * sxx - sx * sx) * (n * syy - sy * sy));
                fitR2 = r * r;
                if (!(slope > 0)) {
                    fail("speed didn't go up with voltage");
                    return false;
                }
                //lag from the slowest steps only, the faster ones look laggy because they run out of torque
                Collections.sort(lagPoints, new java.util.Comparator<double[]>() {
                    public int compare(double[] x, double[] y) {
                        return Double.compare(x[0], y[0]);
                    }
                });
                List<Double> lags = new ArrayList<>();
                for (int k = 0; k < Math.min(4, lagPoints.size()); k++) lags.add(lagPoints.get(k)[1]);
                double timeConstant = lags.isEmpty() ? 0.05 : median(lags), deadTime = median(stepDeadTimes);
                breakaway = Math.max(0, breakaway - 2 * safe(deadTime, 0)); //the friction ramp (2 V/s) kept climbing for a dead time before it saw motion
                double kinetic = clamp(-intercept / slope, 0, breakaway);
                if (fitR2 < 0.9) warnings.add("speed vs voltage isn't very linear (R^2 " + round(fitR2, 3) + "), friction or binding changes with position?");

                params = new Parameters(slope, timeConstant, kinetic, breakaway, deadTime, clamp(0.3 / Math.max(deadTime, 0.01), 2, 25), 0.9, 0.3, 1);
                params.capability(lowTopSpeed, lowAcceleration); //for lining up the full power steps, they measure the real limits
                identified = true;
                stretch = clamp((timeConstant + deadTime + 2 * safe(loopAverage, 0.01)) / 0.1, 1, 3);
                status = "model: " + round(slope / tpd, 1) + " deg/s per V, lag " + round(timeConstant * 1000, 0) + " ms, dead time " + round(deadTime * 1000, 0) + " ms";
                history.add(status + ", friction " + round(kinetic, 2) + " / " + round(breakaway, 2) + " V");
                return true;
            }
        };
    }

    private Stage capability() {

        return new Stage() {
            public String name() {
                return "calculating";
            }

            public boolean run() {

                double acceleration = hardAccelerations.isEmpty() ? params.maxAcceleration : median(hardAccelerations);
                double top = hardSpeeds.isEmpty() ? params.maxSpeed : Collections.min(hardSpeeds);
                params.capability(Math.max(top, params.maxSpeed), acceleration > 0 ? acceleration : params.maxAcceleration);
                double fromRest = hardDeadTimes.isEmpty() ? params.deadTime : median(hardDeadTimes);
                double moving = brakeDelays.isEmpty() ? fromRest : Math.min(fromRest, median(brakeDelays));
                history.add("delay " + round(1000 * moving, 0) + " ms while moving (braking), " + round(1000 * fromRest, 0) + " ms from rest");
                params.deadTime = moving;
                params.startDelay = fromRest;
                params.bandwidth = clamp(0.3 / Math.max(moving + safe(loopAverage, 0.02), 0.01), 2, 25); //the moving delay is what limits how stiff it can be
                if (!coastDecelerations.isEmpty()) {
                    params.maxDeceleration = clamp(median(coastDecelerations), 0.2 * params.maxAcceleration, params.maxAcceleration); //a bad read can't leave it unable to stop
                    history.add("coasts down at " + round(deg(params.maxDeceleration), 0) + " deg/s^2 (it only brakes when reversed)");
                }
                stretch = clamp((params.timeConstant + params.startDelay + 2 * safe(loopAverage, 0.01)) / 0.1, 1, 3);
                status = "top speed " + round(deg(params.maxSpeed), 0) + " deg/s" + (topSpeedReached ? "" : " (at least, still speeding up)") + ", speeds up at " + round(deg(params.maxAcceleration), 0) + " deg/s^2";
                history.add(status);
                stages.addFirst(refine());
                return true;
            }
        };
    }


    private static final double[][] TEST = {
            {0, 0.5}, {30, 1.3}, {-30, 1.6}, {0, 1.3}, {2, 0.7}, {0, 0.7}, {-2, 0.7}, {0, 0.7}, {Double.NaN, 3}, {0, 1}
    };

    private final class TestResult {
        double overshoot, settle, smallError, smallOscillation, holdRms, sineRms, feedforwardCorrection, late;
        boolean saturatedSine;

        boolean ok() {
            return overshoot <= 0.08 && holdRms <= 4 && smallOscillation <= 3;
        }

        //hopping on friction while it holds is bad but it isn't the loop going unstable
        boolean unstable() {
            return overshoot > 0.3 || smallOscillation > 8;
        }

        double score() {
            return sineRms + 10 * overshoot + 2 * settle + smallError + 0.1 * holdRms;
        }
    }

    private Stage refine() {

        return new Stage() {
            int iteration = 0, segment = -1, validations = 0;
            double segmentStart, segmentTarget, lastSign;
            double good = Double.NaN, bad = Double.NaN, soft = Double.NaN, closestScore = Double.POSITIVE_INFINITY;
            Parameters closest = null;
            TestResult closestResult = null;
            boolean validating = false;
            Parameters best = null;
            double bestScore = Double.POSITIVE_INFINITY;
            TestResult r;
            double closeSince = Double.NaN, maxBeyond, lastFar, errorSum, errorCount, oscillations, velocitySquares, velocityCount, sineSquares, sineCount, fbv, vv, ed, dd;
            int saturatedCount;

            public String name() {
                return validating ? "final check" : "tuning loop " + (iteration + 1);
            }

            public boolean run() {

                if (segment < 0) {
                    r = new TestResult();
                    segment = 0;
                    beginSegment();
                }

                double elapsed = t - segmentStart;
                double[] seg = TEST[segment];
                double desired, desiredVelocity = 0;
                if (Double.isNaN(seg[0])) {
                    double w = 2 * Math.PI * 0.7;
                    desired = 25 * tpd * Math.sin(w * elapsed);
                    desiredVelocity = 25 * tpd * w * Math.cos(w * elapsed);
                }
                else {
                    desired = seg[0] * tpd;
                }
                track(desired, desiredVelocity);
                status = (validating ? "checking" : "testing") + " bandwidth " + round(params.bandwidth, 2) + " rad/s";
                measure(seg, elapsed, desired, desiredVelocity);

                if (elapsed >= seg[1] * stretch) {
                    endSegment(seg);
                    segment++;
                    if (segment < TEST.length) {
                        beginSegment();
                        return false;
                    }
                    segment = -1;
                    return decide();
                }
                return false;
            }

            void beginSegment() {

                segmentStart = t;
                segmentTarget = Double.isNaN(TEST[segment][0]) ? 0 : TEST[segment][0] * tpd;
                maxBeyond = 0;
                lastFar = 0;
                errorSum = errorCount = oscillations = velocitySquares = velocityCount = sineSquares = sineCount = fbv = vv = ed = dd = 0;
                saturatedCount = 0;
                lastSign = 0;
                closeSince = Double.NaN;
            }

            void measure(double[] seg, double elapsed, double desired, double desiredVelocity) {

                double error = position - desired;
                double size = Math.abs(segmentTarget - previousTarget());
                if (!Double.isNaN(seg[0]) && size > 10 * tpd) {
                    double direction = Math.signum(segmentTarget - previousTarget());
                    maxBeyond = Math.max(maxBeyond, direction * error);
                    if (Math.abs(error) > tpd) lastFar = elapsed;
                }
                if (!Double.isNaN(seg[0]) && size < 10 * tpd && elapsed > 0.4 * seg[1] * stretch) {
                    errorSum += Math.abs(error);
                    errorCount++;
                    if (Math.abs(error) < 2 * tpd) {
                        double sign = Math.abs(velocity) > 3 * tpd ? Math.signum(velocity) : 0;
                        if (sign != 0 && lastSign != 0 && sign != lastSign) oscillations++;
                        if (sign != 0) lastSign = sign;
                    }
                }
                //vibration = moving after it has settled
                boolean close = !Double.isNaN(seg[0]) && Math.abs(error) < 0.5 * tpd && Math.abs(controller.getReference() - desired) < 1;
                if (!close) closeSince = Double.NaN;
                else if (Double.isNaN(closeSince)) closeSince = elapsed;
                else if (elapsed - closeSince > 0.15 * stretch) {
                    velocitySquares += velocity * velocity;
                    velocityCount++;
                }
                if (Double.isNaN(seg[0]) && elapsed > 0.5) {
                    sineSquares += error * error;
                    sineCount++;
                    double rv = controller.getReferenceVelocity();
                    fbv += (controller.p + controller.i + controller.d) * rv;
                    vv += rv * rv;
                    ed += error * desiredVelocity;
                    dd += desiredVelocity * desiredVelocity;
                    if (Math.abs(controller.volts) >= availableVolts - 1e-6) saturatedCount++;
                }
            }

            double previousTarget() {

                for (int k = segment - 1; k >= 0; k--) if (!Double.isNaN(TEST[k][0])) return TEST[k][0] * tpd;
                return 0;
            }

            void endSegment(double[] seg) {

                double size = Math.abs(segmentTarget - previousTarget());
                if (!Double.isNaN(seg[0]) && size > 10 * tpd) {
                    r.overshoot = Math.max(r.overshoot, maxBeyond / size);
                    r.settle = Math.max(r.settle, lastFar);
                }
                if (errorCount > 0) {
                    r.smallError = Math.max(r.smallError, deg(errorSum / errorCount));
                    r.smallOscillation = Math.max(r.smallOscillation, oscillations);
                }
                if (velocityCount > 0) r.holdRms = Math.max(r.holdRms, deg(Math.sqrt(velocitySquares / velocityCount)));
                else if (!Double.isNaN(seg[0])) r.holdRms = Math.max(r.holdRms, 11); //never settled
                if (sineCount > 0) {
                    r.sineRms = deg(Math.sqrt(sineSquares / sineCount));
                    r.feedforwardCorrection = vv > 0 ? fbv / vv : 0;
                    r.late = dd > 0 ? -ed / dd : 0; //seconds behind the target, error = -late * velocity
                    r.saturatedSine = saturatedCount > 0.1 * sineCount;
                }
            }

            boolean decide() {

                history.add(String.format(java.util.Locale.US, "%s w %.2f K %.0f dead %.0fms: over %.0f%% settle %.2fs small %.2f osc %.0f hold %.1f sine %.2f (late %.0fms)%s", validating ? "check" : "test", params.bandwidth, params.gain, 1000 * params.deadTime, 100 * r.overshoot, r.settle, r.smallError, r.smallOscillation, r.holdRms, r.sineRms, 1000 * r.late, r.ok() ? " ok" : ""));

                if (validating) {
                    validations++;
                    if (r.ok() || validations >= 3) {
                        record(r);
                        return true;
                    }
                    params.bandwidth *= 0.85;
                    return false;
                }

                iteration++;
                double w = params.bandwidth;

                if (r.ok() && r.score() < bestScore) {
                    bestScore = r.score();
                    best = params.copy();
                }

                //overshoot or swinging back and forth = too stiff. Not settling, or hopping on friction while it holds, without
                //either = too soft, and lower would only be worse
                boolean aggressive = r.overshoot > 0.08 || r.smallOscillation > 3;
                if (r.ok()) good = Double.isNaN(good) ? w : Math.max(good, w);
                else if (aggressive || (!Double.isNaN(good) && w > good)) bad = Double.isNaN(bad) ? w : Math.min(bad, w);
                else soft = Double.isNaN(soft) ? w : Math.max(soft, w);
                if (!r.unstable() && r.score() < closestScore) {
                    closestScore = r.score();
                    closest = params.copy();
                    closestResult = r;
                }

                //friction only gets nudged on passing tests: near the edge of stability small errors are the loop, not friction.
                //Above 1 the sliding friction alone beats breakaway and it creeps past the target
                if (r.ok()) {
                    if (r.smallError > 0.4 && r.smallOscillation <= 1) {
                        params.frictionScale = Math.min(1, params.frictionScale + 0.1);
                        params.integralRatio = Math.min(0.8, params.integralRatio * 1.25);
                    }
                    else if (r.smallOscillation > 2) {
                        params.frictionScale = Math.max(0.4, params.frictionScale - 0.2);
                    }
                }

                double next;
                if (!Double.isNaN(good)) next = Double.isNaN(bad) ? 1.3 * good : 0.5 * (good + bad);
                else if (!Double.isNaN(bad) && !Double.isNaN(soft)) next = Math.sqrt(soft * bad);
                else if (!Double.isNaN(bad)) next = 0.65 * bad;
                else next = 1.4 * soft;

                boolean converged = !Double.isNaN(good) && !Double.isNaN(bad) && bad / good < 1.12;
                boolean squeezed = Double.isNaN(good) && !Double.isNaN(soft) && !Double.isNaN(bad) && bad / soft < 1.12;
                if (converged || squeezed || iteration >= 10 || next < 0.5 || next > 40) {
                    if (best == null) {
                        if (closest == null) {
                            fail("every bandwidth overshot or shook, check for binding or a loose encoder");
                            return false;
                        }
                        double gainNow = params.gain;
                        params = closest.copy();
                        params.gain = gainNow;
                        warnings.add("no bandwidth passed every check, kept the closest (" + round(params.bandwidth, 2) + " rad/s)");
                        record(closestResult);
                        return true;
                    }
                    double gainNow = params.gain;
                    params = best.copy();
                    params.gain = gainNow;
                    params.bandwidth = 0.9 * good;
                    validating = true;
                    return false;
                }
                params.bandwidth = clamp(next, 0.5, 40);
                return false;
            }
        };
    }

    private TestResult lastTest;

    private void record(TestResult r) {

        lastTest = r;
        results.put("overshoot_pct", round(100 * r.overshoot, 1));
        results.put("settle_s", round(r.settle, 3));
        results.put("small_step_error_deg", round(r.smallError, 3));
        results.put("hold_vibration_deg_s", round(r.holdRms, 2));
        results.put("tracking_error_deg", round(r.sineRms, 3));
    }


    private Stage waitForA() {

        return new Stage() {
            public String name() {
                return "waiting for A";
            }

            public boolean run() {

                drivingAllowed = false;
                status = "point the turret straight, get ready to drive, then press A (A does nothing else until then)";
                hold(0);
                return startRequested;
            }
        };
    }

    private Stage drivingTune() {

        return new Stage() {
            double headingStart = Double.NaN, start, windowStart, motionTotal, settleUntil = -1;
            boolean swinging;
            final TurretRoute route = new TurretRoute();
            double calmestShake = Double.NaN, lastShake, lastFlips, lastError, lastTrack = Double.NaN, stillSince = Double.NaN, holdTotal;
            boolean caughtUp;
            //shaky holds start a trial at a softer bandwidth. Softer that really shakes less, and holds and tracks no worse, becomes the
            //floor vibration rejection may soften to while it shakes (it softens while moving too); the bandwidth stays what STATIONARY found
            double baseBandwidth = Double.NaN, shakeBandwidth, beforeShake, beforeFlips, beforeError, beforeTrack, trialShake, trialFlips, trialError, trialTrack;
            double floorShake = Double.NaN, floorFlips, floorError, floorTrack;
            int trialJudged = -1, trialUnsettled, trialTracked;
            boolean shakeIsExternal;
            int calmWindows, windows, badWindows;
            double ee, ev, vvSum, fbv, vv, holdSquares, holdErrorSquares, motion, n, holdN, holdTime, stillTime, errorSigns, lastErrorSign;

            public String name() {
                return "driving";
            }

            public boolean run() {

                drivingAllowed = true;
                if (Double.isNaN(headingStart)) {
                    headingStart = heading;
                    start = windowStart = t;
                    controller.reset(position);
                    baseBandwidth = shakeBandwidth = params.bandwidth;
                    params.shakeBandwidth = shakeBandwidth;
                    results.put("shake_bandwidth", round(shakeBandwidth, 2));
                }

                //holds the field direction it started at however much the robot spins, swinging round at the ends like the match aim
                double raw = aimDirection * -(heading - headingStart) * 180 / Math.PI;
                double low = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES + TRAVEL_INSET_DEGREES, high = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES - TRAVEL_INSET_DEGREES;
                double targetSpeed = aimDirection * -omega * 180 / Math.PI;
                //same stop the controller plans: one lead, then 0.8 of the coasting deceleration
                double decel = deg(params.maxDeceleration > 0 ? params.maxDeceleration : params.maxAcceleration > 0 ? params.maxAcceleration : 1000 * tpd);
                double aim = route.update(raw, targetSpeed, deg(position), low, high, params.deadTime + 2 * safe(loopAverage, 0.02), 0.8 * decel, SWING_MARGIN_DEGREES);
                if (route.hasSwung()) swinging = true;
                boolean outside = route.isOutside(low, high);
                double desired = aim * tpd;
                double desiredVelocity = outside ? 0 : targetSpeed * tpd;
                if (swinging && !outside && Math.abs(desired - position) < 5 * tpd) {
                    swinging = false;
                    settleUntil = t + 0.5;
                }
                boolean pinned = swinging || outside || t < settleUntil;
                track(desired, desiredVelocity);

                double error = desired - position;
                boolean still = !pinned && Math.abs(desiredVelocity) < 3 * tpd;
                if (!still) stillSince = Double.NaN;
                else if (Double.isNaN(stillSince)) stillSince = t;
                //shake only counts once the target's been still a while and the turret has caught up: before that it's catching up.
                //After that everything counts, so a soft loop that wanders off doesn't look calm
                if (!still) caughtUp = false;
                else if (t - stillSince >= 0.5 && Math.abs(error) < 2 * tpd) caughtUp = true;
                boolean holding = still && caughtUp;
                if (still && t - stillSince >= 0.5) stillTime += dt;
                if (!holding) lastErrorSign = 0;
                if (!pinned) {
                    if (Math.abs(desiredVelocity) > 15 * tpd) {
                        double rv = controller.getReferenceVelocity();
                        ee += error * error;
                        ev += error * rv;
                        vvSum += rv * rv;
                        fbv += (controller.p + controller.i + controller.d) * rv;
                        vv += rv * rv;
                        motion += dt;
                        n++;
                    }
                    else if (holding) {
                        double shake = velocity - controller.getReferenceVelocity();
                        holdSquares += shake * shake;
                        holdErrorSquares += error * error;
                        holdN++;
                        holdTime += dt;
                        double sign = Math.abs(error) > 0.3 * tpd ? Math.signum(error) : 0;
                        if (sign != 0 && lastErrorSign != 0 && sign != lastErrorSign) errorSigns++;
                        if (sign != 0) lastErrorSign = sign;
                    }
                }

                if (t - windowStart >= 6) {
                    motionTotal += motion;
                    holdTotal += holdTime;
                    windows++;
                    //gain stays what the stationary run measured: feedback lining up with speed here is the start delay at every
                    //reversal, not a wrong gain, and chasing it dragged the gain down 30%
                    double rms = n > 20 ? deg(Math.sqrt(ee / n)) : Double.NaN;
                    double correlation = ev / Math.sqrt(ee * vvSum + 1e-12);
                    if (!Double.isNaN(rms)) results.put("driving_tracking_error_deg", round(rms, 3));
                    //error per speed (s): how fast the driver turned in this window cancels out, the raw error swings 20% with that
                    double lag = n > 20 && vvSum > 0 ? Math.sqrt(ee / vvSum) : Double.NaN;
                    String note;
                    boolean changed = false, unsettled = false;
                    Boolean softerHelped = null;
                    if (trialJudged >= 0 && !Double.isNaN(lag)) {
                        trialTrack += lag;
                        trialTracked++;
                    }
                    if (windows <= 1) note = " (start-up, not judged)"; //first window has the start-up jump
                    else if (holdTime < 0.5) {
                        note = " (no hold, not judged)";
                        //it was still but never caught up: softer can't hold
                        if (trialJudged >= 0 && stillTime >= 0.5 && ++trialUnsettled >= 2) softerHelped = false;
                    }
                    else {
                        double holdRms = deg(Math.sqrt(holdSquares / holdN)), holdError = deg(Math.sqrt(holdErrorSquares / holdN));
                        double flips = errorSigns / holdTime;
                        calmestShake = Double.isNaN(calmestShake) ? holdRms : Math.min(calmestShake, holdRms);
                        //slow loops shake more anyway, so only clearly worse than its calmest counts
                        boolean bad = flips > 3 || holdRms > Math.max(6, 1.6 * calmestShake);
                        note = bad ? " shaky" : " calm";
                        if (trialJudged >= 0) {
                            trialShake += holdRms;
                            trialFlips += flips;
                            trialError += holdError;
                            if (++trialJudged >= 2) {
                                double s = trialShake / 2, f = trialFlips / 2, e = trialError / 2, k = trialTracked > 0 ? trialTrack / trialTracked : Double.NaN;
                                boolean tracks = Double.isNaN(k) || Double.isNaN(beforeTrack) || k <= 1.05 * beforeTrack;
                                softerHelped = (s < 0.8 * beforeShake || beforeFlips > 3 && f <= 3) && e < beforeError && tracks;
                                if (softerHelped) {
                                    floorShake = s;
                                    floorFlips = f;
                                    floorError = e;
                                    floorTrack = k;
                                }
                            }
                        }
                        else {
                            double next = 0.85 * shakeBandwidth;
                            boolean canTry = !shakeIsExternal && next >= Math.max(2, 0.5 * baseBandwidth);
                            badWindows = bad && canTry ? badWindows + 1 : 0;
                            unsettled = bad && canTry;
                            //only ever softer, and only while it shakes: raising it here chased the lag the start delay makes at every reversal and it hunted
                            if (badWindows >= 2) {
                                boolean fromFloor = !Double.isNaN(floorShake);
                                beforeShake = fromFloor ? floorShake : 0.5 * (lastShake + holdRms);
                                beforeFlips = fromFloor ? floorFlips : 0.5 * (lastFlips + flips);
                                beforeError = fromFloor ? floorError : 0.5 * (lastError + holdError);
                                beforeTrack = fromFloor ? floorTrack : Double.isNaN(lastTrack) ? lag : Double.isNaN(lag) ? lastTrack : 0.5 * (lastTrack + lag);
                                params.bandwidth = next;
                                trialShake = trialFlips = trialError = trialTrack = 0;
                                trialJudged = trialUnsettled = trialTracked = 0;
                                badWindows = 0;
                                changed = true;
                                note += String.format(java.util.Locale.US, ", trying %.2f to see if softer shakes less", next);
                            }
                        }
                        lastShake = holdRms;
                        lastFlips = flips;
                        lastError = holdError;
                        results.put("driving_hold_vibration_deg_s", round(holdRms, 2));
                        note = String.format(java.util.Locale.US, " hold %.1f deg/s, off by %.2f deg over %.1f s, flips %.1f/s,", holdRms, holdError, holdTime, flips) + note;
                    }
                    if (softerHelped != null) {
                        if (softerHelped) shakeBandwidth = params.bandwidth;
                        else shakeIsExternal = true;
                        note += softerHelped ? String.format(java.util.Locale.US, ", softer helped: rejection may soften to %.2f while it shakes", shakeBandwidth) : ", softer didn't help (the shake isn't the loop's), rejection won't soften below " + round(shakeBandwidth, 2);
                        params.bandwidth = baseBandwidth;
                        params.shakeBandwidth = shakeBandwidth;
                        results.put("shake_bandwidth", round(shakeBandwidth, 2));
                        trialJudged = -1;
                        changed = true;
                    }
                    if (trialJudged < 0 && !changed) lastTrack = lag;
                    if (changed || trialJudged >= 0 || unsettled) calmWindows = 0;
                    else if (windows > 1 && holdTime >= 0.5) calmWindows++;
                    history.add(String.format(java.util.Locale.US, "drive window: K %.0f w %.2f error %.2f deg corr %.2f", params.gain, params.bandwidth, rms, correlation) + note);
                    windowStart = t;
                    ee = ev = vvSum = fbv = vv = holdSquares = holdErrorSquares = motion = n = holdN = holdTime = stillTime = errorSigns = 0;
                    lastErrorSign = 0;
                }

                status = "drive like a match, spin and turn as much as you want, and stop turning for a second or two now and then (that's when it checks the shake)" + (pinned ? " (swinging around the end)" : "") + ". turning time " + round(motionTotal + motion, 0) + "/20 s, held still " + round(holdTotal + holdTime, 0) + " s, settled windows " + calmWindows + "/3";
                return motionTotal >= 20 && calmWindows >= 3 && t - start >= 40;
            }
        };
    }

    private void finish() {

        done = true;
        status = "done, values below";
        Parameters p = params;
        results.put("gain", round(p.gain, 2));
        results.put("time_constant", round(p.timeConstant, 4));
        results.put("kinetic_friction", round(p.kineticFriction, 3));
        results.put("breakaway_friction", round(p.breakawayFriction, 3));
        results.put("dead_time", round(p.deadTime, 4));
        results.put("bandwidth", round(p.bandwidth, 2));
        results.put("damping", round(p.damping, 3));
        results.put("integral_ratio", round(p.integralRatio, 3));
        results.put("friction_scale", round(p.frictionScale, 3));
        if (p.maxSpeed > 0 && !Double.isInfinite(p.maxSpeed)) results.put("max_speed", round(p.maxSpeed, 1));
        if (p.startDelay > 0) results.put("start_delay", round(p.startDelay, 4));
        if (p.maxDeceleration > 0 && !Double.isInfinite(p.maxDeceleration)) results.put("max_deceleration", round(p.maxDeceleration, 1));
        if (p.maxAcceleration > 0 && !Double.isInfinite(p.maxAcceleration)) results.put("max_acceleration", round(p.maxAcceleration, 1));
        params.apply(controller);
        results.put("kp", controller.kp);
        results.put("kd", controller.kd);
        results.put("ki", controller.ki);
        results.put("direction_sign", directionSign);
        if (!Double.isNaN(fitR2)) results.put("fit_r2", round(fitR2, 4));
        if (!Double.isNaN(restNoise)) results.put("rest_velocity_noise_deg_s", round(deg(restNoise), 3));
        results.put("speed_deg_s_per_volt", round(p.gain / tpd, 2));
    }


    private static double median(List<Double> values) {

        if (values.isEmpty()) return Double.NaN;
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted.get(sorted.size() / 2);
    }

    private static double safe(double v, double fallback) {
        return Double.isNaN(v) ? fallback : v;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static double round(double v, int digits) {

        double s = Math.pow(10, digits);
        return Math.round(v * s) / s;
    }
}
