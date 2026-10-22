package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Turret control for continuous-rotation servos with an external angle sensor (design D-4, D-5).
 *
 * <pre>
 *   routing   : pick the equivalent target angle within soft limits nearest the turret (with hysteresis)
 *   reference : online time-optimal generator (accel-limited "braking curve") that tracks a MOVING target,
 *               with the target rate (bearing rate − robot yaw rate) as a velocity feedforward.
 *               A reference governor keeps r within maxLag of the turret, so saturation never builds up overshoot.
 *   control   : u = kS·sgn + kV·r' + kA·( r'' + kp·(r − θ^) + kd·(r' − ω^) − f^ )      (ADRC, ESO-based)
 *   park      : once settled on a stationary target, the output is released to 0 (+ preload) so the
 *               turret rests instead of hunting through the servo deadband and gear lash
 *   preload   : optional opposing bias between the two servos to take up backlash at hold
 * </pre>
 *
 * Angles are in turret radians relative to the robot's forward axis (CCW positive). Output is the servo
 * power for each side. Allocation-free.
 */
public final class TurretController {

    public static final class Config {
        /** Identified servo model in power units per turret rad/s. Priors assume ~170°/s free speed ❓. */
        public MotorFeedforward model = new MotorFeedforward(0.06, 0.31, 0.020, 0.05);
        public double observerBandwidth = 35.0;     // rad/s
        public double trackingBandwidth = 9.0;      // wc [rad/s] → kp = wc², kd = 2·zeta·wc
        public double trackingDamping = 1.0;        // zeta
        /** Reference speed limit (rad/s). Set to ~92% of the identified free speed (1 − kS)/kV so that some
         *  authority is left for corrections: 2.8 rad/s ≈ 160°/s for a ≈172°/s servo ❓. */
        public double maxVelocity = 2.8;
        public double maxAcceleration = 25.0;       // rad/s²
        /** Below this error (rad) the braking curve becomes linear to avoid chatter near the target. */
        public double linearZone = Math.toRadians(1.0);
        public double minAngle = Math.toRadians(-200);
        public double maxAngle = Math.toRadians(200);
        /** Extra travel required before switching to another equivalent target angle. */
        public double routingHysteresis = Math.toRadians(20);
        /** Reference governor: max allowed |r − θ^| (rad). */
        public double maxReferenceLag = Math.toRadians(8);
        /** Park when |error| < holdBand and the target is still; unpark at 2× holdBand. */
        public double holdBand = Math.toRadians(0.35);
        public double stillTargetRate = Math.toRadians(2);   // rad/s
        /** Anti-backlash opposing bias per servo (power). 0 = off until tested for servo heat ❓. */
        public double preloadPower = 0.0;
        /** Sensor sanity: a reading further than this from the prediction counts as a glitch (rad). */
        public double glitchThreshold = Math.toRadians(15);
        public int glitchesToFault = 3;
        /** Soft-limit margin beyond which only motion back toward the range is allowed. */
        public double limitMargin = Math.toRadians(3);
        /**
         * Stuck detection: an unplugged or slipping encoder (constant reading) or a jam/hard stop looks like
         * "pushing hard, not moving". Latches a fault (servos off) to protect the servos and wiring.
         */
        public double stuckPower = 0.35;
        public double stuckRate = Math.toRadians(3);
        public double stuckTime = 0.5;
    }

    private final Config cfg;
    private final ExtendedStateObserver eso;

    private double targetAngle;      // wrapped target (rad)
    private double targetRate;       // rad/s
    private double routedTarget;     // chosen equivalent within limits
    private boolean routedValid;
    private boolean targetReachable = true;

    private double r, rDot, rDdot;   // reference
    private double appliedPower;
    private double leftPower, rightPower;
    private boolean enabled;
    private boolean parked;
    private boolean faulted;
    private int consecutiveGlitches;

    private double now;
    private boolean haveTime;

    public TurretController(Config cfg) {
        this.cfg = cfg;
        this.eso = new ExtendedStateObserver(cfg.observerBandwidth);
        this.eso.setMaxInnovation(cfg.glitchThreshold);
    }

    /**
     * @param angleRobotFrame desired turret angle relative to robot forward (any wrap; rad)
     * @param rateRobotFrame  its time derivative (rad/s): target bearing rate − robot yaw rate
     */
    public void setTarget(double angleRobotFrame, double rateRobotFrame) {
        if (!MathUtil.isFinite(angleRobotFrame)) return;
        targetAngle = MathUtil.wrapPi(angleRobotFrame);
        targetRate = MathUtil.isFinite(rateRobotFrame) ? rateRobotFrame : 0;
        if (!enabled) {
            enabled = true;
            r = eso.position();
            rDot = 0;
            routedValid = false;
        }
    }

    public void disable() {
        enabled = false;
        parked = false;
    }

    /** Clears a latched sensor fault (call after the cause is fixed, e.g. re-homing). */
    public void clearFault() {
        faulted = false;
        consecutiveGlitches = 0;
        stuckTimer = 0;
        eso.invalidate();
    }

    /**
     * @param measuredAngle continuous turret angle from the sensor (rad); NaN = no reading
     * @param t             monotonic time (s)
     */
    public void update(double measuredAngle, double t) {
        double dt = haveTime ? t - now : 0;
        now = t;
        haveTime = true;

        // --- sensing & health ---
        if (!MathUtil.isFinite(measuredAngle)) {
            fault();
            return;
        }
        boolean wasInit = eso.isInitialized();
        if (wasInit && dt > 0) {
            double predicted = eso.position() + eso.velocity() * dt;
            if (Math.abs(measuredAngle - predicted) > cfg.glitchThreshold) {
                if (++consecutiveGlitches >= cfg.glitchesToFault) {
                    fault();
                    return;
                }
            } else {
                consecutiveGlitches = 0;
            }
        }
        MotorFeedforward m = cfg.model;
        eso.update(measuredAngle, m.acceleration(appliedPower, eso.velocity()), dt);
        if (!wasInit || faulted) {
            r = eso.position();
            rDot = 0;
            setOutputs(0);
            return;
        }
        if (!(dt > 0) || dt > 0.5) {
            setOutputs(0);
            return;
        }
        double theta = eso.position();
        double omega = eso.velocity();

        if (!enabled) {
            r = theta;
            rDot = 0;
            setOutputs(0);
            return;
        }

        // --- routing ---
        route(theta);

        // --- reference generator (tracks the moving routed target) ---
        double vt = targetReachable ? targetRate : 0;
        double e = routedTarget - r;
        double ae = Math.abs(e);
        double vRel;
        if (ae > cfg.linearZone) {
            vRel = Math.signum(e) * Math.sqrt(2 * cfg.maxAcceleration * (ae - 0.5 * cfg.linearZone));
        } else {
            vRel = e * Math.sqrt(cfg.maxAcceleration / (0.5 * cfg.linearZone));
        }
        double vDes = MathUtil.clamp(vt + vRel, -cfg.maxVelocity, cfg.maxVelocity);
        double newRDot = MathUtil.approach(rDot, vDes, cfg.maxAcceleration * dt);
        rDdot = (newRDot - rDot) / dt;
        r += 0.5 * (rDot + newRDot) * dt;
        rDot = newRDot;
        if (r > cfg.maxAngle) { r = cfg.maxAngle; rDot = Math.min(rDot, 0); }
        if (r < cfg.minAngle) { r = cfg.minAngle; rDot = Math.max(rDot, 0); }
        // reference governor
        double lag = r - theta;
        if (Math.abs(lag) > cfg.maxReferenceLag) {
            r = theta + Math.signum(lag) * cfg.maxReferenceLag;
        }

        // --- park logic (no hunting) ---
        double err = routedTarget - theta;
        boolean targetStill = Math.abs(vt) < cfg.stillTargetRate;
        if (parked) {
            if (!targetStill || Math.abs(err) > 2 * cfg.holdBand) parked = false;
        } else if (targetStill && Math.abs(err) < cfg.holdBand && Math.abs(omega) < cfg.stillTargetRate) {
            parked = true;
        }
        if (parked) {
            r = theta;
            rDot = 0;
            setOutputs(0);
            return;
        }

        // --- ADRC control law ---
        double kp = cfg.trackingBandwidth * cfg.trackingBandwidth;
        double kd = 2 * cfg.trackingDamping * cfg.trackingBandwidth;
        double correction = kp * (r - theta) + kd * (rDot - omega) - eso.disturbance();
        double frictionDir = Math.abs(rDot) > m.signWidth
                ? Math.signum(rDot)
                : MathUtil.smoothSign(r - theta, cfg.holdBand);
        double u = m.kS * frictionDir + m.kV * rDot + m.kA * (rDdot + correction);

        // soft limits: beyond the margin only allow motion back into range
        if (theta > cfg.maxAngle + cfg.limitMargin && u > 0) u = 0;
        if (theta < cfg.minAngle - cfg.limitMargin && u < 0) u = 0;

        // Uses the RAW measurement, not the observer: with a frozen reading the observer's model still
        // predicts motion for a while, which would delay detection.
        if (Math.abs(u) > cfg.stuckPower) {
            if (stuckTimer == 0) stuckStartAngle = measuredAngle;
            stuckTimer += dt;
            if (Math.abs(measuredAngle - stuckStartAngle) > cfg.stuckRate * cfg.stuckTime) stuckTimer = 0;
        } else {
            stuckTimer = 0;
        }
        if (stuckTimer > cfg.stuckTime) {
            fault();
            return;
        }
        setOutputs(u);
    }

    private double stuckTimer, stuckStartAngle;

    private void route(double theta) {
        double best = Double.NaN;
        double bestCost = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double cand = targetAngle + k * MathUtil.TAU;
            if (cand < cfg.minAngle || cand > cfg.maxAngle) continue;
            double cost = Math.abs(cand - theta);
            if (routedValid && Math.abs(cand - routedTarget) > Math.PI) cost += cfg.routingHysteresis;
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }
        if (Double.isNaN(best)) {
            // target lies in the unreachable gap (limit span < 360°): go to the nearest limit
            double toMin = Math.abs(MathUtil.wrapPi(targetAngle - cfg.minAngle));
            double toMax = Math.abs(MathUtil.wrapPi(targetAngle - cfg.maxAngle));
            best = toMin < toMax ? cfg.minAngle : cfg.maxAngle;
            targetReachable = false;
        } else {
            targetReachable = true;
        }
        routedTarget = best;
        routedValid = true;
    }

    private void setOutputs(double u) {
        u = MathUtil.clamp(MathUtil.isFinite(u) ? u : 0, -1, 1);
        appliedPower = u;
        double p = Math.min(cfg.preloadPower, Math.max(0, 1 - Math.abs(u)));
        leftPower = u + p;
        rightPower = u - p;
    }

    private void fault() {
        faulted = true;
        parked = false;
        appliedPower = 0;
        leftPower = 0;
        rightPower = 0;
    }

    /** Re-homing: tells the turret its true current angle (rad) without disturbing velocity estimates. */
    public void rehome(double trueAngle) {
        double delta = trueAngle - eso.position();
        eso.shiftPosition(delta);
        r += delta;
    }

    // --- outputs & status ---
    public double leftPower() { return leftPower; }
    public double rightPower() { return rightPower; }
    public double appliedPower() { return appliedPower; }
    public double angle() { return eso.position(); }
    public double rate() { return eso.velocity(); }
    public double disturbance() { return eso.disturbance(); }
    public double reference() { return r; }
    public double referenceRate() { return rDot; }
    public double routedTarget() { return routedTarget; }
    public boolean isTargetReachable() { return targetReachable; }
    public boolean isParked() { return parked; }
    public boolean isFaulted() { return faulted; }
    public boolean isEnabled() { return enabled; }

    /** Angular error to the routed target (rad). */
    public double error() { return routedTarget - eso.position(); }

    /** Rate error to the target rate (rad/s). */
    public double rateError() { return targetRate - eso.velocity(); }

    public Config config() { return cfg; }
}
