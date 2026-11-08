package org.firstinspires.ftc.teamcode.Systems.Turret;

public class TurretRoute {

    private double unwrapped = Double.NaN, outsideSince = Double.NaN, lastSwing = Double.NEGATIVE_INFINITY;
    private boolean swung;

    public void reset() {

        unwrapped = Double.NaN;
        outsideSince = Double.NaN;
        lastSwing = Double.NEGATIVE_INFINITY;
    }

    public double update(double aim, double aimRate, double turretNow, double min, double max, double lead, double stopDecel, double margin) {

        if (Double.isNaN(unwrapped)) unwrapped = nearest(aim, turretNow, min, max);
        else unwrapped += Math.IEEEremainder(aim - unwrapped, 360);
        double mid = 0.5 * (min + max);
        boolean upper = unwrapped > mid;
        double outward = upper ? aimRate : -aimRate;
        double room = upper ? max - unwrapped : unwrapped - min;
        double stop = outward > 0 ? outward * lead + outward * outward / (2 * stopDecel) : 0;
        boolean racing = outward > 0 && room < stop + margin;
        boolean parked = room < -margin && -outward < -room - margin;
        swung = racing || parked;
        if (swung) unwrapped += upper ? -360 : 360;
        return clamp(unwrapped, min, max);
    }

    public double update(double aim, double aimRate, double turretNow, double min, double max, double margin, double now) {

        if (Double.isNaN(unwrapped)) unwrapped = nearest(aim, turretNow, min, max);
        else unwrapped += Math.IEEEremainder(aim - unwrapped, 360);
        boolean upper = unwrapped > 0.5 * (min + max);
        double outside = upper ? unwrapped - max : min - unwrapped;
        double inward = upper ? -aimRate : aimRate;
        boolean past = outside > margin && inward * RETURN_SECONDS < outside - margin;
        if (!past) outsideSince = Double.NaN;
        else if (Double.isNaN(outsideSince)) outsideSince = now;
        boolean farPast = outside > FAR_PAST;
        swung = farPast || past && now - outsideSince >= SWING_DELAY && now - lastSwing >= SWING_COOLDOWN;
        if (swung) {
            unwrapped += upper ? -360 : 360;
            lastSwing = now;
            outsideSince = Double.NaN;
        }
        return clamp(unwrapped, min, max);
    }

    public static final double RETURN_SECONDS = 1, SWING_DELAY = 0.25, SWING_COOLDOWN = 1, FAR_PAST = 45;

    public boolean isOutside(double min, double max) {
        return unwrapped < min || unwrapped > max;
    }

    public boolean hasSwung() {
        return swung;
    }

    private static double nearest(double aim, double turretNow, double min, double max) {

        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = aim + 360 * k;
            if (candidate < min || candidate > max) continue;
            if (Double.isNaN(best) || Math.abs(candidate - turretNow) < Math.abs(best - turretNow)) best = candidate;
        }
        return Double.isNaN(best) ? aim - 360 * Math.floor((aim - min) / 360) : best;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
