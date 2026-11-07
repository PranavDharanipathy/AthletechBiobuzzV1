package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import java.util.Locale;

/// findings.md for a hypertuning session: what each tuner got to, and principle candidates for an agent (see SYSTEM_EVOLUTION.md).
/// A knob pinned at a bound or a tuner that can't move its cost says the structure is the limit, not the numbers. No hardware calls
public final class Findings {

    private Findings() {}

    public static String markdown(String session, double seconds, FlywheelHyperTuner flywheel, TurretHyperTuner turret) {

        StringBuilder b = new StringBuilder();
        b.append("# Shooter hypertuning findings: ").append(session).append("\n\n");
        b.append(String.format(Locale.US, "%.0f s played, %d bursts scored, %d turret moves scored.\n\n", seconds, flywheel.bursts, turret.movesScored));
        StringBuilder principles = new StringBuilder();
        for (OnlineTuner t : new OnlineTuner[] {flywheel.stability, flywheel.shots, turret.moves}) tuner(b, principles, t);
        b.append("## Rotation negation\n\n");
        b.append(String.format(Locale.US, "Rate feedforward scale %.3f (1 = cancel all of the spin). Last window: bias %.2f deg (+ = lags the spin), RMS %.2f deg.\n\n", turret.rateScale, turret.lastTrackBias, turret.lastTrackRms));
        if (turret.rateScale >= 1.29) principles.append("- **Turret can't keep up with spins even fully fed** (scale pinned at 1.3): speed or acceleration limits it. Candidate: lead the aim by the spin (the SOTM idea for rotation), or re-measure TURRET_MAX_SPEED (logs showed 460 deg/s against 384 measured).\n");
        if (turret.rateScale <= 0.71) principles.append("- **Turret overdoes the spin** (scale pinned at 0.7): the omega it's fed leads reality. Candidate: check Pinpoint heading-rate scale and latency, or delay the rate feedforward by the measured lag.\n");
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
            case "bandwidth":
                return top ? "The turret wants to be stiffer: shake isn't the limit, so raise the cap or re-run the autotuner." : "Shake sets stiffness: isolate the turret from the flywheel (mechanical) or notch the flywheel frequency out of the error.";
            case "brake_degrees":
                return top ? "Softer cruise braking keeps winning: the plan's speed is optimistic, re-measure top speed." : "Hard flip braking wins: the fade isn't needed.";
            case "max_speed_fraction":
            case "acceleration_fraction":
                return top ? "It wants to plan faster than allowed: re-measure TURRET_MAX_SPEED / ACCELERATION with the autotuner." : "Slower plans win: the turret can't do what it measured (battery, drag), re-run the autotuner under match load.";
            case "resync_degrees":
                return top ? "Resyncs hurt: the plan matches the turret well, the feature can go." : "Resync as tight as allowed wins: the model is far off, re-run the autotuner.";
            case "landing":
                return top ? "The landing brake beats the plan's stop on this robot: make TURRET_LANDING_ENABLED default." : "The landing brake loses: measure reverse braking (an autotuner stage) before trying it again.";
            default:
                return "";
        }
    }

    static String fmt(double v) {
        return String.format(Locale.US, "%.4g", v);
    }
}
