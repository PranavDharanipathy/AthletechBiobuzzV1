package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import java.util.Locale;

public final class Findings {

    private Findings() {}

    public static String markdown(String session, double seconds, FlywheelHyperTuner flywheel) {

        StringBuilder b = new StringBuilder();
        b.append("# Shooter hypertuning findings: ").append(session).append("\n\n");
        b.append(String.format(Locale.US, "%.0f s played, %d bursts scored.\n\n", seconds, flywheel.bursts));
        StringBuilder principles = new StringBuilder();
        for (OnlineTuner t : new OnlineTuner[] {flywheel.stability, flywheel.shots}) tuner(b, principles, t);
        b.append("## Principle candidates\n\n");
        b.append(principles.length() == 0 ? "None: every tuner moved its cost and nothing is pinned at a bound.\n" : principles.toString());
        return b.toString();
    }

    private static void tuner(StringBuilder b, StringBuilder principles, OnlineTuner t) {

        b.append("## ").append(t.name).append("\n\n");
        double change = t.firstBaseline > 0 ? 100 * (1 - t.baseline / t.firstBaseline) : Double.NaN;
        b.append(String.format(Locale.US, "Cost %.4g -> %.4g (%.0f%% better), %d trials, %d kept.\n\n", t.firstBaseline, t.baseline, change, t.trials, t.keeps));
        b.append("| knob | now | bounds | kept | kept at a bound |\n|---|---|---|---|---|\n");
        for (OnlineTuner.Knob k : t.knobs) {
            b.append(String.format(Locale.US, "| %s | %.5g | %.4g..%.4g | %d | %d |\n", k.name, k.value, k.min, k.max, k.kept, k.atBound));
            boolean pinned = k.atBound >= 2 || (k.kept >= 2 && (k.value <= k.min || k.value >= k.max));
            if (pinned) principles.append("- **").append(t.name).append(": ").append(k.name).append(" wants past ").append(k.value >= k.max ? "its top" : "its bottom").append("** (").append(fmt(k.value)).append("). ").append(hint(k)).append('\n');
        }
        b.append('\n');
        if (t.trials >= 12 && t.keeps == 0) principles.append("- **").append(t.name).append(": no knob moved the cost** in ").append(t.trials).append(" trials. Something these knobs can't reach sets it (noise, mechanics, timing): look at the worst windows in loop.csv and add a principle for that.\n");
    }

    private static String hint(OnlineTuner.Knob k) {

        boolean top = k.value >= k.max;
        switch (k.name) {
            case "feed_boost_volts":
                return top ? "A flat boost isn't enough: learn each ball's load as a profile from the blocker opening (iterative learning control) and feed it forward." : "Boosting while feeding only overspeeds: drops come after contact, the kick is the lever.";
            case "shot_kick_volts":
                return top ? "The kick saturates: the motor is at full power after a ball. Next levers: more speed headroom before the shot, or a second motor." : "Kicks hurt: shot detection may be firing late or falsely, check FLYWHEEL_SHOT_DETECT_DROP against loop.csv.";
            case "shot_kick_time":
                return top ? "Recovery wants a long push: replace the kick with a learned recovery profile." : "Only the first instant matters: detect contact sooner (raw velocity, not filtered).";
            case "max_p":
                return top ? "Recovery is P-limited at full power: the battery/motor is the limit, space the feed out by the recovery time instead." : "Big P recovery overshoots: add a deceleration limit to P.";
            case "kp_close":
            case "kp_far":
                return top ? "Drift wins: an integral or a faster load observer is the next principle." : "Noise wins: a better velocity estimate (filter, encoder) is the next principle.";
            case "kd":
                return top ? "Damping wants more than D can give on a noisy velocity: model-based damping (acceleration from current) instead." : "D only adds noise: drop it.";
            default:
                return "";
        }
    }

    static String fmt(double v) {
        return String.format(Locale.US, "%.4g", v);
    }
}
