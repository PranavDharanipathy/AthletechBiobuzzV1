package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

public class OnlineTuner {

    public static final class Knob {

        public final String name;
        public final double min, max, minStep;
        public double value, step;
        public int kept, atBound;

        public Knob(String name, double value, double min, double max, double step, double minStep) {

            this.name = name;
            this.min = min;
            this.max = max;
            this.value = clamp(value, min, max);
            this.step = step;
            this.minStep = minStep;
        }
    }

    public static final class Trial {

        public String knob = "";
        public double from, to, cost, baseline;
        public boolean kept;
    }

    public final String name;
    public final Knob[] knobs;
    //paired: a trial alternates the new value and the old one sample by sample (ABBA) and judges the differences, so a battery
    //running down or a motor warming up over the session can't make a change look better or worse (2028-05-15: the baseline
    //drifted 9.1 -> 11.3 while 6 changes were kept)
    public boolean paired = true;
    public double margin = 0.03, grow = 1.3, shrink = 0.6, confidence = 1.5;
    public int samplesPerTrial = 3, maxSamplesPerTrial = 9, lastSamples;
    public int trials, keeps;
    //cost spread / cost, squared, learned from every trial (one trial's few samples alone can't say)
    public double relativeVariance = 0.01;
    public double baseline = Double.NaN, firstBaseline = Double.NaN;
    public final Trial last = new Trial();

    private int index, phase, samples;
    private double sum, sumSquares, saved, baselineSamples, noiseWeight = 2;
    private boolean onOld, pendingOld;
    private double pending = Double.NaN, diffSum, diffSquares, oldSum, oldSquares, newSum;
    private int pairs;

    public OnlineTuner(String name, int samplesPerTrial, Knob... knobs) {

        this.name = name;
        this.samplesPerTrial = Math.max(1, samplesPerTrial);
        this.maxSamplesPerTrial = 3 * this.samplesPerTrial;
        this.knobs = knobs;
    }

    private void learnNoise(double mean) {

        if (samples < 2 || !(Math.abs(mean) > 1e-12)) return;
        double variance = Math.max(0, (sumSquares - samples * mean * mean) / (samples - 1));
        noiseWeight = Math.min(noiseWeight + 1, 30);
        relativeVariance += (variance / (mean * mean) - relativeVariance) / noiseWeight;
    }

    public Knob knob(String name) {

        for (Knob k : knobs) if (k.name.equals(name)) return k;
        throw new IllegalArgumentException(name);
    }

    //what to run now: the knob on trial reads its old value on the old half of a pair
    public double value(String name) {

        Knob k = knob(name);
        return paired && onOld && phase != 0 && k == knobs[index] ? saved : k.value;
    }

    public String trying() {
        return phase == 0 ? "" : knobs[index].name + (paired ? (onOld ? " (old)" : " (new)") : "");
    }

    public boolean submit(double cost) {

        if (Double.isNaN(cost) || Double.isInfinite(cost) || knobs.length == 0) return false;
        if (paired) return submitPaired(cost);
        sum += cost;
        sumSquares += cost * cost;
        if (++samples < samplesPerTrial) return false;
        double mean = sum / samples;
        Knob k = knobs[index];
        if (phase == 0) {
            learnNoise(mean);
            baseline = mean;
            baselineSamples = samples;
            sum = sumSquares = samples = 0;
            if (Double.isNaN(firstBaseline)) firstBaseline = mean;
            startTrial(k, 1);
            return false;
        }
        //how many samples depends on the noise: stop once it's clearly better or clearly not, else keep measuring up to the max
        double gain = baseline > 1e-12 ? (baseline - mean) / baseline : baseline - mean;
        double error = confidence * Math.sqrt(relativeVariance * (1.0 / samples + 1.0 / Math.max(1, baselineSamples)));
        boolean better = gain > margin && gain > error;
        boolean notBetter = gain + error <= margin;
        if (!better && !notBetter && samples < maxSamplesPerTrial) return false;
        learnNoise(mean);
        lastSamples = samples;
        sum = sumSquares = samples = 0;
        trials++;
        last.knob = k.name;
        last.from = saved;
        last.to = k.value;
        last.cost = mean;
        last.baseline = baseline;
        last.kept = better;
        if (last.kept) {
            keeps++;
            k.kept++;
            if (k.value <= k.min || k.value >= k.max) k.atBound++;
            baseline = mean;
            baselineSamples = lastSamples;
            k.step *= grow;
            nextKnob();
        }
        else if (phase == 1) {
            k.value = saved;
            startTrial(k, -1);
        }
        else {
            k.value = saved;
            k.step = Math.max(k.minStep, k.step * shrink);
            nextKnob();
        }
        return true;
    }

    private boolean submitPaired(double cost) {

        //nothing on trial yet: that sample ran the old values, start the first trial
        if (phase == 0) {
            startTrial(knobs[index], 1);
            return false;
        }
        //ABBA: new-old, old-new, new-old, ...
        if (Double.isNaN(pending)) {
            pending = cost;
            pendingOld = onOld;
            onOld = !onOld;
            return false;
        }
        double newCost = pendingOld ? cost : pending, oldCost = pendingOld ? pending : cost;
        pending = Double.NaN;
        pairs++;
        double d = newCost - oldCost;
        diffSum += d;
        diffSquares += d * d;
        oldSum += oldCost;
        oldSquares += oldCost * oldCost;
        newSum += newCost;
        double old = oldSum / pairs, mean = diffSum / pairs;
        baseline = old;
        if (Double.isNaN(firstBaseline)) firstBaseline = old;
        if (pairs < samplesPerTrial) return false;
        double sampleVariance = pairs > 1 ? Math.max(0, (diffSquares - pairs * mean * mean) / (pairs - 1)) : 0;
        //a few pairs can look quiet by luck: never trust less spread than the noise learned so far
        double prior = 2 * relativeVariance * old * old;
        double error = confidence * Math.sqrt(Math.max(sampleVariance, prior) / pairs);
        double scale = Math.max(Math.abs(old), 1e-12);
        boolean better = -mean > margin * scale && -mean > error;
        boolean notBetter = -mean + error <= margin * scale;
        if (!better && !notBetter && pairs < maxSamplesPerTrial) return false;
        if (pairs > 1 && Math.abs(old) > 1e-12) {
            double oldVariance = Math.max(0, (oldSquares - pairs * old * old) / (pairs - 1));
            noiseWeight = Math.min(noiseWeight + 1, 30);
            relativeVariance += (oldVariance / (old * old) - relativeVariance) / noiseWeight;
        }
        Knob k = knobs[index];
        lastSamples = 2 * pairs;
        trials++;
        last.knob = k.name;
        last.from = saved;
        last.to = k.value;
        last.cost = newSum / pairs;
        last.baseline = old;
        last.kept = better;
        pairs = 0;
        diffSum = diffSquares = oldSum = oldSquares = newSum = 0;
        onOld = false;
        if (better) {
            keeps++;
            k.kept++;
            if (k.value <= k.min || k.value >= k.max) k.atBound++;
            baseline = last.cost;
            k.step *= grow;
            nextKnob();
        }
        else if (phase == 1) {
            k.value = saved;
            startTrial(k, -1);
        }
        else {
            k.value = saved;
            k.step = Math.max(k.minStep, k.step * shrink);
            nextKnob();
        }
        return true;
    }

    private void startTrial(Knob k, int direction) {

        saved = k.value;
        double to = clamp(saved + direction * k.step, k.min, k.max);
        if (to == saved) {
            if (direction > 0) {
                startTrial(k, -1);
                return;
            }
            k.step = Math.max(k.minStep, k.step * shrink);
            nextKnob();
            return;
        }
        k.value = to;
        phase = direction > 0 ? 1 : 2;
    }

    private void nextKnob() {

        index = (index + 1) % knobs.length;
        if (index == 0 && !paired) phase = 0;
        else startTrial(knobs[index], 1);
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
