package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Flywheel speed control: identified-model feedforward + model-assisted ADRC (design D-1, D-2).
 *
 * <pre>
 *   observer : ESO on encoder position → speed w^, total disturbance f^ (rad/s²)
 *   reference: r ramps toward the target no faster than the motor can physically follow at the
 *              present battery voltage / current limit (no overshoot after saturation)
 *   control  : u = kS·sgn(r) + kV·r + kA·r'                (inverse model, feedforward)
 *                + kA·( wc·(r − w^) − f^ )                  (first-order error dynamics + disturbance cancel)
 *                + kA·(−d_shot)                              (anticipated ball-contact disturbance)
 *   bounds   : |u| ≤ V_batt·headroom and |u − kV·w^| ≤ R·I_max (model-based current limit)
 * </pre>
 *
 * The observer is fed the <i>applied</i> (bounded) voltage, so there is nothing to wind up. There is no
 * integrator and no bang-bang. Units: rad, rad/s, V, s. Allocation-free.
 */
public final class FlywheelController {

    public static final class Config {
        public MotorFeedforward model = new MotorFeedforward(0.25, 0.0190, 0.0170, 2.0);
        /**
         * ESO bandwidth wo [rad/s]. Higher = faster disturbance rejection, more quantization noise.
         * Simulation study (28 cpr, 8–15 ms loop): wo=45 → ±1.1 V command ripple; wo=30 → ±0.43 V with a
         * shot still 17σ above the noise. 30 chosen.
         */
        public double observerBandwidth = 30.0;
        /** Closed-loop error bandwidth wc [rad/s]. Keep ≤ wo/3. */
        public double trackingBandwidth = 10.0;
        /** Fraction of battery voltage usable (driver drop, margin). */
        public double voltageHeadroom = 0.97;
        /** R·I_max [V]: limits |u − kV·w| so that motor current ≤ I_max. Infinity = off. */
        public double currentLimitVolts = 13.0;
        /** Reference may use at most this fraction of the physically available acceleration. */
        public double referenceAccelMargin = 0.85;
        /** Ready when |target − w^| ≤ this fraction of target (or the absolute floor), held for readyHoldTime. */
        public double readyToleranceFraction = 0.015;
        public double readyToleranceFloor = 4.0;        // rad/s
        public double readyHoldTime = 0.05;            // s
        /** Disturbance below −this (rad/s²) is treated as a ball contact (noise σ ≈ 14 at wo=30, shot ≈ −240). */
        public double shotDetectThreshold = 120.0;
        /**
         * Energy banking: once a feed is notified, raise the reference by this fraction of the learned per-ball
         * speed loss until contact ends. The dip then lands near the target instead of below it. Recovery is
         * power-limited, so storing energy beforehand is the only way to shorten it. The exit speed this
         * produces is handled by the speed-adaptive hood.
         */
        public double bankFraction = 0.5;
        public double shotRefractoryTime = 0.12;       // s
        /** Anticipation: ball contact starts this long after notifyFeed() and lasts contactDuration. */
        public double feedToContactDelay = 0.08;       // s ❓ measured from logs
        public double contactDuration = 0.04;          // s ❓
        /** Initial guess and learning rate for the speed lost per ball (rad/s). */
        public double initialShotSpeedLoss = 25.0;
        public double shotLossLearnRate = 0.3;
        /** Feeds whose measured loss is below this fraction of the expected loss are flagged as "no ball". */
        public double noBallFraction = 0.3;
        /** Innovation clamp for encoder glitches [rad]. */
        public double maxInnovation = 3.0;
        /**
         * Stall / dead-encoder detection: an unplugged encoder reads a constant, so speed ≈ 0 while we push
         * hard. Commanding more than stallVolts with speed below stallSpeed for stallTime latches a fault
         * (output 0) until the target is set to 0. That protects the motor and tells the driver.
         */
        public double stallVolts = 6.0;
        public double stallSpeed = 15.0;    // rad/s
        public double stallTime = 0.6;      // s
    }

    private final Config cfg;
    private final ExtendedStateObserver eso;

    private double target;          // rad/s requested
    private double reference;       // shaped reference r
    private double referenceRate;   // r'
    private double appliedVolts;    // last bounded command
    private boolean enabled;
    private boolean faulted;

    // readiness
    private double inToleranceTime;

    // shot handling
    private double now;
    private double lastShotTime = Double.NEGATIVE_INFINITY;
    private int shotCount;
    private double learnedShotLoss;
    private double feedTime = Double.NaN;
    private double shotLossIntegral;  // ∫(f^ + d_shot) dt over the learning window
    private boolean feedWithoutBall;
    private boolean shotArmed;
    private boolean stalled;
    private double stallTimer;
    private double anticipatedAccel;  // d_shot currently applied (rad/s², negative while active)

    public FlywheelController(Config cfg) {
        this.cfg = cfg;
        this.eso = new ExtendedStateObserver(cfg.observerBandwidth);
        this.eso.setMaxInnovation(cfg.maxInnovation);
        this.learnedShotLoss = cfg.initialShotSpeedLoss;
    }

    /** Sets the speed target [rad/s]. 0 = coast (no braking current). */
    public void setTarget(double radPerSec) {
        if (!MathUtil.isFinite(radPerSec)) radPerSec = 0;
        radPerSec = Math.max(0, radPerSec);
        if (radPerSec == 0) {
            stalled = false; // commanding stop clears a stall fault (driver retry)
            stallTimer = 0;
        }
        if (!enabled && radPerSec > 0) {
            // bumpless start: begin the reference at the current measured speed
            reference = eso.velocity();
            referenceRate = 0;
        }
        enabled = radPerSec > 0;
        target = radPerSec;
    }

    /**
     * Tells the controller that a ball was just released into the flywheel, so the expected energy loss can
     * be cancelled at contact instead of corrected afterwards.
     */
    public void notifyFeed() {
        feedTime = now;
        shotLossIntegral = 0;
    }

    /**
     * Runs one control step.
     *
     * @param positionRad  encoder position converted to flywheel radians (NaN = sensor fault)
     * @param t            monotonic time [s]
     * @param batteryVolts filtered battery voltage
     * @return voltage to apply (convert with {@link VoltageCompensator#toPower(double)})
     */
    public double update(double positionRad, double t, double batteryVolts) {
        double dt = t - now;
        boolean firstStep = !eso.isInitialized();
        now = t;

        if (!MathUtil.isFinite(positionRad)) {
            faulted = true;
            appliedVolts = 0;
            return 0;
        }
        if (stalled) {
            faulted = true;
            appliedVolts = 0;
            eso.update(positionRad, 0, dt);
            return 0;
        }
        faulted = false;

        MotorFeedforward m = cfg.model;

        // 1) Observer: predict with the voltage actually applied over the last interval.
        double knownAccel = m.acceleration(appliedVolts, eso.velocity()) + anticipatedAccel;
        eso.update(positionRad, knownAccel, dt);
        if (firstStep || !(dt > 0) || dt > 0.5) {
            appliedVolts = 0;
            return 0; // need two samples (or a sane dt) before acting
        }
        double w = eso.velocity();
        double f = eso.disturbance();

        // 2) Shot bookkeeping (uses this step's estimate).
        updateShotTracking(f, dt);

        if (!enabled) {
            reference = w;
            referenceRate = 0;
            appliedVolts = 0;
            inToleranceTime = 0;
            return 0;
        }

        // 3) Bounds available right now.
        double vAvail = Math.max(0, batteryVolts) * cfg.voltageHeadroom;
        double uHigh = Math.min(vAvail, m.kV * w + cfg.currentLimitVolts);
        double uLow = Math.max(-vAvail, m.kV * w - cfg.currentLimitVolts);
        uLow = Math.max(uLow, 0.0); // never drive backwards: a flywheel only needs coast/brake via 0 V

        // 4) Reference shaping: never ask for more acceleration than the motor can deliver.
        double sgn = MathUtil.smoothSign(reference, m.signWidth);
        double accUp = (uHigh - m.kS * sgn - m.kV * reference) / m.kA;
        double accDown = (uLow - m.kS * sgn - m.kV * reference) / m.kA;
        accUp = Math.max(0, accUp) * cfg.referenceAccelMargin;
        accDown = Math.min(0, accDown) * cfg.referenceAccelMargin;
        double effectiveTarget = target;
        if (!Double.isNaN(feedTime) && now - feedTime < cfg.feedToContactDelay + cfg.contactDuration) {
            effectiveTarget += cfg.bankFraction * learnedShotLoss;
        }
        if (effectiveTarget < reference && w < reference) {
            // Coming down while the wheel is already below the reference (e.g. the ball just took the banked
            // energy): never command a deceleration the plant has already done. Snap the reference to the
            // measured speed (bounded by the new target).
            reference = Math.max(effectiveTarget, w);
            referenceRate = 0;
        }
        double desiredRate = (effectiveTarget - reference) / dt;
        referenceRate = MathUtil.clamp(desiredRate, accDown, accUp);
        reference += referenceRate * dt;

        // 5) Anticipated shot disturbance (known-disturbance feedforward).
        anticipatedAccel = 0;
        if (!Double.isNaN(feedTime)) {
            double since = now - feedTime;
            if (since >= cfg.feedToContactDelay && since < cfg.feedToContactDelay + cfg.contactDuration) {
                anticipatedAccel = -learnedShotLoss / cfg.contactDuration;
            }
        }

        // 6) Control law.
        double u = m.calculate(reference, referenceRate)
                + m.kA * (cfg.trackingBandwidth * (reference - w) - f - anticipatedAccel);
        u = MathUtil.clamp(u, uLow, uHigh);
        if (!MathUtil.isFinite(u)) u = 0;
        appliedVolts = u;

        stallTimer = (u > cfg.stallVolts && Math.abs(w) < cfg.stallSpeed) ? stallTimer + dt : 0;
        if (stallTimer > cfg.stallTime) {
            stalled = true;
            appliedVolts = 0;
            return 0;
        }

        // 7) Readiness.
        double tol = Math.max(cfg.readyToleranceFloor, cfg.readyToleranceFraction * target);
        boolean inTol = Math.abs(target - w) <= tol && Math.abs(target - reference) <= tol;
        inToleranceTime = inTol ? inToleranceTime + dt : 0;

        return u;
    }

    private void updateShotTracking(double f, double dt) {
        // Unanticipated detection (balls fed without notifyFeed). Gated on being at speed (spin-up model
        // error looks like a disturbance), one count per event (re-arm only after the disturbance has
        // decayed), and suppressed during a notified feed (counted from its measured energy loss below).
        boolean atSpeed = enabled && Math.abs(target - reference) <= Math.max(cfg.readyToleranceFloor,
                cfg.readyToleranceFraction * target);
        if (!shotArmed && -f < cfg.shotDetectThreshold / 3 && now - lastShotTime > cfg.shotRefractoryTime) {
            shotArmed = true;
        }
        if (shotArmed && atSpeed && Double.isNaN(feedTime) && -f > cfg.shotDetectThreshold) {
            lastShotTime = now;
            shotCount++;
            shotArmed = false;
        }
        // Learning window for a notified feed: integrate the total disturbance, including the part we
        // anticipated (which the observer does not see, because it was in the model).
        if (!Double.isNaN(feedTime)) {
            double since = now - feedTime;
            double winStart = cfg.feedToContactDelay - 0.03;
            double winEnd = cfg.feedToContactDelay + cfg.contactDuration + 0.20;
            if (since >= winStart && since <= winEnd) {
                shotLossIntegral += (f + anticipatedAccel) * dt;
            } else if (since > winEnd) {
                double measuredLoss = -shotLossIntegral;
                if (measuredLoss < cfg.noBallFraction * learnedShotLoss) {
                    feedWithoutBall = true; // fed, but nothing hit the wheel: jam or empty
                } else {
                    feedWithoutBall = false;
                    learnedShotLoss += cfg.shotLossLearnRate * (measuredLoss - learnedShotLoss);
                    shotCount++;
                    lastShotTime = now;
                }
                feedTime = Double.NaN;
                shotArmed = false; // let the post-shot disturbance decay before re-arming detection
            }
        }
    }

    public boolean isReady() {
        return enabled && !faulted && inToleranceTime >= cfg.readyHoldTime;
    }

    /** Resets the observer (e.g. at OpMode start). */
    public void reset() {
        reference = 0;
        referenceRate = 0;
        appliedVolts = 0;
        enabled = false;
        target = 0;
        inToleranceTime = 0;
        feedTime = Double.NaN;
        anticipatedAccel = 0;
        eso.invalidate(); // re-seeded from the next measurement
    }

    public double velocity() { return eso.velocity(); }
    public double disturbance() { return eso.disturbance(); }
    public double reference() { return reference; }
    public double target() { return target; }
    public double appliedVolts() { return appliedVolts; }
    public boolean isFaulted() { return faulted; }
    public boolean isStalled() { return stalled; }
    public int shotCount() { return shotCount; }
    public double learnedShotLoss() { return learnedShotLoss; }
    public boolean lastFeedHadNoBall() { return feedWithoutBall; }
    public double timeSinceLastShot() { return now - lastShotTime; }
    public Config config() { return cfg; }
}
