package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Battery-voltage tracking and volts → motor power conversion (design decision D-3).
 *
 * <p>Reading the hub voltage sensor costs an I/O transaction, so the caller samples it at a limited rate
 * ({@link #shouldSample(double)}). Samples outside a sane range are rejected, so a glitch reading of 0 V
 * cannot command 100% power. Between samples, residual sag is absorbed by the controllers' disturbance
 * observers.
 */
public final class VoltageCompensator {

    public static final double MIN_SANE_VOLTS = 6.0;
    public static final double MAX_SANE_VOLTS = 16.0;

    private final double samplePeriod;
    private final double filterTau;
    private final double nominalVolts;

    private double filtered = Double.NaN;
    private double lastSampleTime = Double.NEGATIVE_INFINITY;
    private int rejectedSamples = 0;

    /**
     * @param samplePeriod seconds between voltage reads (e.g. 0.05)
     * @param filterTau    low-pass time constant in seconds (e.g. 0.05)
     * @param nominalVolts value assumed before the first valid sample (e.g. 12.5)
     */
    public VoltageCompensator(double samplePeriod, double filterTau, double nominalVolts) {
        this.samplePeriod = samplePeriod;
        this.filterTau = filterTau;
        this.nominalVolts = nominalVolts;
    }

    public boolean shouldSample(double now) {
        return now - lastSampleTime >= samplePeriod;
    }

    public void addSample(double volts, double now) {
        double dt = now - lastSampleTime;
        lastSampleTime = now;
        if (!MathUtil.isFinite(volts) || volts < MIN_SANE_VOLTS || volts > MAX_SANE_VOLTS) {
            rejectedSamples++;
            return;
        }
        if (Double.isNaN(filtered)) {
            filtered = volts;
        } else {
            filtered += MathUtil.lowPassAlpha(MathUtil.isFinite(dt) ? dt : samplePeriod, filterTau) * (volts - filtered);
        }
    }

    /** Best estimate of the battery voltage available to the motor drivers. */
    public double volts() {
        return Double.isNaN(filtered) ? nominalVolts : filtered;
    }

    /** Converts a desired motor voltage into a power command in [-1, 1]. */
    public double toPower(double desiredVolts) {
        return MathUtil.clamp(desiredVolts / volts(), -1.0, 1.0);
    }

    public int rejectedSamples() { return rejectedSamples; }
    public boolean hasSample() { return !Double.isNaN(filtered); }
}
