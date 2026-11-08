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
    public double margin = 0.03, grow = 1.3, shrink = 0.6;
    public int samplesPerTrial = 3;
    public int trials, keeps;
    public double baseline = Double.NaN, firstBaseline = Double.NaN;
    public final Trial last = new Trial();

    private int index, phase, samples;
    private double sum, saved;

    public OnlineTuner(String name, int samplesPerTrial, Knob... knobs) {

        this.name = name;
        this.samplesPerTrial = Math.max(1, samplesPerTrial);
        this.knobs = knobs;
    }

    public Knob knob(String name) {

        for (Knob k : knobs) if (k.name.equals(name)) return k;
        throw new IllegalArgumentException(name);
    }

    public double value(String name) {
        return knob(name).value;
    }

    public String trying() {
        return phase == 0 ? "" : knobs[index].name;
    }

    public boolean submit(double cost) {

        if (Double.isNaN(cost) || Double.isInfinite(cost) || knobs.length == 0) return false;
        sum += cost;
        if (++samples < samplesPerTrial) return false;
        double mean = sum / samples;
        sum = samples = 0;
        Knob k = knobs[index];
        if (phase == 0) {
            baseline = mean;
            if (Double.isNaN(firstBaseline)) firstBaseline = mean;
            startTrial(k, 1);
            return false;
        }
        trials++;
        last.knob = k.name;
        last.from = saved;
        last.to = k.value;
        last.cost = mean;
        last.baseline = baseline;
        last.kept = mean < baseline * (1 - margin) - 1e-12;
        if (last.kept) {
            keeps++;
            k.kept++;
            if (k.value <= k.min || k.value >= k.max) k.atBound++;
            baseline = mean;
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
        if (index == 0) phase = 0;
        else startTrial(knobs[index], 1);
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
