package org.firstinspires.ftc.teamcode.Utility;

public class VoltageCompensator {

    public static final double MIN_SANE_VOLTAGE = 8.0;
    public static final double MAX_SANE_VOLTAGE = 14.5;

    private double samplePeriod;
    private double filterTau;
    private final double nominalVolts;

    private double filtered = Double.NaN;
    private double lastSampleTime = Double.NEGATIVE_INFINITY;
    private double lastValidSampleTime = Double.NEGATIVE_INFINITY;
    private int rejectedSamples = 0;

    public VoltageCompensator(double samplePeriod, double filterTau, double nominalVolts) {

        if (!Double.isFinite(samplePeriod) || samplePeriod < 0 || !Double.isFinite(filterTau) || filterTau < 0 || !Double.isFinite(nominalVolts) || nominalVolts < MIN_SANE_VOLTAGE || nominalVolts > MAX_SANE_VOLTAGE) {
            throw new IllegalArgumentException("Invalid voltage sampling/filter configuration");
        }
        this.samplePeriod = samplePeriod;
        this.filterTau = filterTau;
        this.nominalVolts = nominalVolts;
    }

    public boolean setFilter(double samplePeriod, double filterTau) {

        if (!Double.isFinite(samplePeriod) || samplePeriod < 0 || !Double.isFinite(filterTau) || filterTau < 0) {
            return false;
        }
        this.samplePeriod = samplePeriod;
        this.filterTau = filterTau;
        return true;
    }

    public boolean shouldSample(double now) {
        return Double.isFinite(now) && now - lastSampleTime >= samplePeriod;
    }

    public void addSample(double volts, double now) {

        if (!Double.isFinite(now) || now < lastSampleTime) {
            rejectedSamples++;
            return;
        }
        lastSampleTime = now;
        if (!Double.isFinite(volts) || volts < MIN_SANE_VOLTAGE || volts > MAX_SANE_VOLTAGE) {
            rejectedSamples++;
            return;
        }
        if (Double.isNaN(filtered)) {
            filtered = volts;
        }
        else {
            double dt = now - lastValidSampleTime;
            double alpha = filterTau == 0 ? 1 : -Math.expm1(-dt / filterTau);
            filtered += alpha * (volts - filtered);
        }
        lastValidSampleTime = now;
    }

    public double volts() {
        return Double.isNaN(filtered) ? nominalVolts : filtered;
    }

    public double toPower(double desiredVolts) {

        if (!Double.isFinite(desiredVolts)) return 0;
        return Math.max(-1, Math.min(desiredVolts / volts(), 1));
    }

    public int rejectedSamples() { return rejectedSamples; }
    public boolean hasSample() { return !Double.isNaN(filtered); }
}
