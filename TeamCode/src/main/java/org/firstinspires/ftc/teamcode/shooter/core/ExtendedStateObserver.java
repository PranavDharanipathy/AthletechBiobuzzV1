package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Third-order discrete Extended State Observer (ESO) driven by a position measurement.
 *
 * <p>States: x1 = position, x2 = velocity, x3 = "total disturbance" acceleration, i.e. everything
 * the model does not explain: friction drift, ball contact, battery sag, model error.
 *
 * <p>Plant assumed by the observer: x1' = x2, x2' = a_known + x3, x3' = 0, where a_known is the
 * acceleration the caller's model predicts from the input that was <b>actually applied</b> (after
 * saturation). Feeding the applied input is what makes the controllers built on this observer free of
 * windup.
 *
 * <p>Discretization: "current estimator" form (Miklosovic, Radke, Gao 2006). The gains are recomputed
 * from the measured dt every update so that the observer error poles always sit at
 * z = exp(-wo*dt) (triple). That is stable for every dt > 0, so loop-time jitter cannot destabilize it.
 *
 * <p>Allocation-free.
 */
public final class ExtendedStateObserver {

    private double bandwidth;          // wo [rad/s]
    private double maxInnovation = Double.POSITIVE_INFINITY;

    private double x1, x2, x3;
    private boolean initialized = false;
    private double lastInnovation;

    public ExtendedStateObserver(double bandwidth) {
        setBandwidth(bandwidth);
    }

    public void setBandwidth(double bandwidth) {
        if (!(bandwidth > 0)) throw new IllegalArgumentException("bandwidth must be > 0");
        this.bandwidth = bandwidth;
    }

    /**
     * Limits how far a single measurement can pull the estimate (in position units). This rejects
     * encoder glitches and wrap errors without disturbing normal operation. Set it a few times larger
     * than the largest legitimate prediction error per loop.
     */
    public void setMaxInnovation(double maxInnovation) {
        this.maxInnovation = maxInnovation;
    }

    public void reset(double position, double velocity, double disturbance) {
        x1 = position;
        x2 = velocity;
        x3 = disturbance;
        initialized = true;
    }

    /**
     * @param measuredPosition position measurement (same units as the estimate)
     * @param knownAccel       model-predicted acceleration from the applied input
     * @param dt               seconds since the previous update (ignored if not positive and finite)
     */
    public void update(double measuredPosition, double knownAccel, double dt) {
        if (!MathUtil.isFinite(measuredPosition)) return;
        if (!initialized) {
            reset(measuredPosition, 0, 0);
            return;
        }
        if (!(dt > 0) || !MathUtil.isFinite(dt) || !MathUtil.isFinite(knownAccel)) return;

        double a = knownAccel + x3;
        double p1 = x1 + dt * x2 + 0.5 * dt * dt * a;
        double p2 = x2 + dt * a;
        double p3 = x3;

        double beta = Math.exp(-bandwidth * dt);
        double omb = 1.0 - beta;
        double l1 = 1.0 - beta * beta * beta;
        double l2 = 1.5 * omb * omb * (1.0 + beta) / dt;
        double l3 = omb * omb * omb / (dt * dt);

        double e = MathUtil.clamp(measuredPosition - p1, -maxInnovation, maxInnovation);
        lastInnovation = e;

        x1 = p1 + l1 * e;
        x2 = p2 + l2 * e;
        x3 = p3 + l3 * e;
    }

    /** Forgets the state; the next measurement re-seeds the observer. */
    public void invalidate() {
        initialized = false;
    }

    /** Overwrites the position estimate (e.g. after re-homing) while keeping velocity and disturbance. */
    public void shiftPosition(double delta) {
        x1 += delta;
    }

    public double position() { return x1; }
    public double velocity() { return x2; }
    public double disturbance() { return x3; }
    public double lastInnovation() { return lastInnovation; }
    public boolean isInitialized() { return initialized; }
    public double bandwidth() { return bandwidth; }

    // Gain accessors (exposed for verification tests).
    public static double gainL1(double wo, double dt) {
        double b = Math.exp(-wo * dt);
        return 1.0 - b * b * b;
    }

    public static double gainL2(double wo, double dt) {
        double b = Math.exp(-wo * dt);
        double omb = 1.0 - b;
        return 1.5 * omb * omb * (1.0 + b) / dt;
    }

    public static double gainL3(double wo, double dt) {
        double b = Math.exp(-wo * dt);
        double omb = 1.0 - b;
        return omb * omb * omb / (dt * dt);
    }
}
