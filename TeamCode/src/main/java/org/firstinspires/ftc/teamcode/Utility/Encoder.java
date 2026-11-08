package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;

    private Direction direction;
    public Encoder(DcMotorEx motor) {

        this.motor = motor;

        this.direction = Direction.FORWARD;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    private int offset = 0;

    //hub keeps the old count between OpModes
    public void reset() {
        reset(0);
    }

    public void reset(int position) {

        if (motor != null) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode);
        }
        offset = position;
        resetVelocityFilter();
    }

    public int getCurrentPosition() {

        int multiplier = direction.getMultiplier();
        return motor.getCurrentPosition() * multiplier + offset;
    }

    public double getInternalVelocity() {

        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }
    private static final int FILTERS = 7;
    private static final int NOISE_SAMPLES = 64;

    private final double[] filterBandwidths = new double[FILTERS];
    private double scoreMemory, staleTime;

    {
        setVelocityFilterParameters(0.5, 32, 0.9, 0.05);
    }

    public void setVelocityFilterParameters(double minBandwidth, double maxBandwidth, double scoreMemory, double staleTime) {

        double ratio = Math.pow(maxBandwidth / minBandwidth, 1.0 / (FILTERS - 1));
        for (int k = 0; k < FILTERS; k++) filterBandwidths[k] = minBandwidth * Math.pow(ratio, k);
        this.scoreMemory = scoreMemory;
        this.staleTime = staleTime;
    }

    private final double[] filterVelocity = new double[FILTERS], filterAcceleration = new double[FILTERS];
    private final double[] p00 = new double[FILTERS], p01 = new double[FILTERS], p11 = new double[FILTERS];
    private final double[] score = new double[FILTERS], weight = new double[FILTERS];

    private final double[] secondDifferences = new double[NOISE_SAMPLES], sortBuffer = new double[NOISE_SAMPLES];
    private int secondDifferenceCount = 0, secondDifferenceIndex = 0;
    private double smallestStep = Double.POSITIVE_INFINITY;

    private boolean filterStarted = false;
    private double lastFilterTime, lastRawChangeTime, samplePeriod = 0.01;
    private double raw1 = Double.NaN, raw2 = Double.NaN;
    private double noiseVariance = 100;

    private double filteredVelocity = 0, filteredAcceleration = 0, filterBandwidth = 0;

    public void resetVelocityFilter() {

        filterStarted = false;
        secondDifferenceCount = 0;
        secondDifferenceIndex = 0;
        smallestStep = Double.POSITIVE_INFINITY;
        raw1 = raw2 = Double.NaN;
        noiseVariance = 100;
    }

    public double updateFilteredVelocity() {
        return updateFilteredVelocity(Clock.seconds(), getInternalVelocity());
    }

    public double updateFilteredVelocity(double now, double raw) {

        if (Double.isNaN(raw) || Double.isInfinite(raw)) return filteredVelocity;

        if (!filterStarted || now - lastFilterTime <= 0 || now - lastFilterTime > 0.5) {
            startFilter(now, raw);
            return filteredVelocity;
        }

        double dt = now - lastFilterTime;
        lastFilterTime = now;

        boolean newReading = raw != raw1 || now - lastRawChangeTime >= staleTime;
        if (newReading) {
            samplePeriod += 0.05 * (now - lastRawChangeTime - samplePeriod);
            lastRawChangeTime = now;
            updateNoise(raw);
        }

        double best = Double.NEGATIVE_INFINITY;
        int bestIndex = 0;

        for (int k = 0; k < FILTERS; k++) {

            double omega = 2 * Math.PI * filterBandwidths[k];
            double q = omega * omega * omega * omega * noiseVariance * samplePeriod;

            filterVelocity[k] += filterAcceleration[k] * dt;
            double a00 = p00[k] + 2 * dt * p01[k] + dt * dt * p11[k] + q * dt * dt * dt / 3;
            double a01 = p01[k] + dt * p11[k] + q * dt * dt / 2;
            double a11 = p11[k] + q * dt;
            p00[k] = a00;
            p01[k] = a01;
            p11[k] = a11;

            if (newReading) {
                double s = p00[k] + noiseVariance;
                double innovation = raw - filterVelocity[k];
                double k0 = p00[k] / s, k1 = p01[k] / s;

                filterVelocity[k] += k0 * innovation;
                filterAcceleration[k] += k1 * innovation;
                p11[k] -= k1 * p01[k];
                p01[k] *= 1 - k0;
                p00[k] *= 1 - k0;

                double logLikelihood = -0.5 * (Math.log(2 * Math.PI * s) + innovation * innovation / s);
                score[k] = scoreMemory * score[k] + Math.max(logLikelihood, -50);
            }

            if (score[k] > best) {
                best = score[k];
                bestIndex = k;
            }
        }

        double total = 0;
        for (int k = 0; k < FILTERS; k++) {
            weight[k] = Math.exp(score[k] - best);
            total += weight[k];
        }

        filteredVelocity = 0;
        filteredAcceleration = 0;
        filterBandwidth = 0;
        for (int k = 0; k < FILTERS; k++) {
            weight[k] /= total;
            filteredVelocity += weight[k] * filterVelocity[k];
            filteredAcceleration += weight[k] * filterAcceleration[k];
            filterBandwidth += weight[k] * filterBandwidths[k];
        }

        for (int k = 0; k < FILTERS; k++) {
            double gap = filterVelocity[k] - filterVelocity[bestIndex];
            if (gap * gap > 9 * (p00[k] + p00[bestIndex] + noiseVariance)) {
                filterVelocity[k] = filterVelocity[bestIndex];
                filterAcceleration[k] = filterAcceleration[bestIndex];
            }
        }

        return filteredVelocity;
    }

    private void startFilter(double now, double raw) {

        filterStarted = true;
        lastFilterTime = now;
        lastRawChangeTime = now;

        for (int k = 0; k < FILTERS; k++) {
            filterVelocity[k] = raw;
            filterAcceleration[k] = 0;
            p00[k] = noiseVariance;
            p01[k] = 0;
            p11[k] = 1e6;
            score[k] = 0;
        }

        filteredVelocity = raw;
        filteredAcceleration = 0;
        raw2 = raw1;
        raw1 = raw;
    }

    private void updateNoise(double raw) {

        if (raw == raw1) return;

        if (!Double.isNaN(raw1)) smallestStep = Math.min(smallestStep, Math.abs(raw - raw1));

        if (!Double.isNaN(raw2)) {
            secondDifferences[secondDifferenceIndex] = Math.abs(raw - 2 * raw1 + raw2);
            secondDifferenceIndex = (secondDifferenceIndex + 1) % NOISE_SAMPLES;
            if (secondDifferenceCount < NOISE_SAMPLES) secondDifferenceCount++;
        }

        raw2 = raw1;
        raw1 = raw;

        if (secondDifferenceCount < 16) return;

        System.arraycopy(secondDifferences, 0, sortBuffer, 0, secondDifferenceCount);
        java.util.Arrays.sort(sortBuffer, 0, secondDifferenceCount);
        double sigma = 1.4826 * sortBuffer[secondDifferenceCount / 2];

        double floor = Double.isInfinite(smallestStep) ? 1 : Math.max(1, smallestStep * smallestStep / 12);
        noiseVariance = Math.max(floor, sigma * sigma / 6);
    }

    public double getFilteredVelocity() {
        return filteredVelocity;
    }

    public double getFilteredAcceleration() {
        return filteredAcceleration;
    }

    public double getFilterBandwidth() {
        return filterBandwidth;
    }

    public double getVelocityNoise() {
        return Math.sqrt(noiseVariance);
    }
}
