package org.firstinspires.ftc.teamcode.Tuners;

import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

public final class FlywheelVoltageFilterFit {

    public static final double MAX_READ_LOAD = 0.10;
    public static final double MIN_SAMPLE_PERIOD = 0.005;

    public static final double REFERENCE_WINDOW = 0.06;

    public static final double MIN_TRUSTED_SAG_VOLTS = 0.2;

    private static final int MIN_SAMPLES = 200;
    private static final int MIN_WINDOW_SAMPLES = 5;
    private static final double MIN_DURATION = 5;

    private static final double[] CANDIDATE_TAUS = {
            0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5
    };

    public static final class Result {
        public final double samplePeriod;
        public final double filterTau;
        public final double rmsVolts;
        public final double currentRmsVolts;
        public final double sagVolts;

        Result(double samplePeriod, double filterTau, double rmsVolts, double currentRmsVolts, double sagVolts) {

            this.samplePeriod = samplePeriod;
            this.filterTau = filterTau;
            this.rmsVolts = rmsVolts;
            this.currentRmsVolts = currentRmsVolts;
            this.sagVolts = sagVolts;
        }

        public boolean isTrusted() {
            return sagVolts >= MIN_TRUSTED_SAG_VOLTS;
        }
    }

    private FlywheelVoltageFilterFit() {
    }

    public static Result fit(double[] times, double[] volts, int count, double averageReadSeconds, double currentPeriod, double currentTau) {

        if (count < MIN_SAMPLES || times[count - 1] - times[0] < MIN_DURATION) return null;

        double[] reference = centeredAverage(times, volts, count);

        double period = Math.max(MIN_SAMPLE_PERIOD, averageReadSeconds / MAX_READ_LOAD);
        period = Math.ceil(period * 1000) / 1000;

        double bestTau = CANDIDATE_TAUS[0];
        double bestRms = Double.POSITIVE_INFINITY;
        for (double tau : CANDIDATE_TAUS) {
            double rms = replayRms(times, volts, reference, count, period, tau);
            if (rms < bestRms) {
                bestRms = rms;
                bestTau = tau;
            }
        }

        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (int k = 0; k < count; k++) {

            min = Math.min(min, reference[k]);
            max = Math.max(max, reference[k]);
        }

        return new Result(period, bestTau, bestRms, replayRms(times, volts, reference, count, currentPeriod, currentTau), max - min);
    }

    static double replayRms(double[] times, double[] volts, double[] reference, int count, double period, double tau) {

        VoltageCompensator filter = new VoltageCompensator(period, tau, 12);
        double start = times[0] + REFERENCE_WINDOW;
        double sum = 0;
        int scored = 0;

        for (int k = 0; k < count; k++) {
            if (filter.shouldSample(times[k])) filter.addSample(volts[k], times[k]);
            if (times[k] < start || times[k] > times[count - 1] - REFERENCE_WINDOW) continue;

            double difference = filter.volts() - reference[k];
            sum += difference * difference;
            scored++;
        }
        return scored == 0 ? Double.POSITIVE_INFINITY : Math.sqrt(sum / scored);
    }

    static double[] centeredAverage(double[] times, double[] volts, int count) {

        double meanDt = (times[count - 1] - times[0]) / (count - 1);
        double halfWindow = Math.max(REFERENCE_WINDOW, MIN_WINDOW_SAMPLES * meanDt) / 2;

        double[] average = new double[count];
        double sum = 0;
        int low = 0;
        int high = 0;

        for (int k = 0; k < count; k++) {
            while (high < count && times[high] <= times[k] + halfWindow) {
                sum += volts[high];
                high++;
            }
            while (times[low] < times[k] - halfWindow) {
                sum -= volts[low];
                low++;
            }
            average[k] = sum / (high - low);
        }
        return average;
    }
}
