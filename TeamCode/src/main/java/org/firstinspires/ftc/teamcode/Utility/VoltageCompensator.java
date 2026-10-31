package org.firstinspires.ftc.teamcode.Utility;

public class VoltageCompensator {

    public static final double MIN_SANE_VOLTAGE = 8.0;
    public static final double MAX_SANE_VOLTAGE = 14.5;

    private final double samplePeriod;
    private final double filterTau;
    private final double nominalVolts;

    private double filtered = Double.NaN;
    private double lastSampleTime = Double.NEGATIVE_INFINITY;
    private int rejectedSamples = 0;

    /**
     * @param samplePeriod seconds between voltage reads
     * @param filterTau tuned low-pass filter constant in seconds
     * @param nominalVolts value assumed before the first valid sample
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
        if (!MathUtil.isFinite(volts) || volts < MIN_SANE_VOLTAGE || volts > MAX_SANE_VOLTAGE) {
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
        return MathUtil.clamp(desiredVolts / volts(), -1d, 1d);
    }

    public int rejectedSamples() { return rejectedSamples; }
    public boolean hasSample() { return !Double.isNaN(filtered); }
}