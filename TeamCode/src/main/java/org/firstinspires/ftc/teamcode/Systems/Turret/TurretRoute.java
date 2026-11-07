package org.firstinspires.ftc.teamcode.Systems.Turret;

/// which turn of the aim the turret goes to (deg from home), so the robot can spin forever. Heading for an end faster than
/// the turret can stop before it, the aim swings a full turn around and the turret waits at the other end till it comes in.
/// No hardware calls, the sims run it
public class TurretRoute {

    private double unwrapped = Double.NaN;
    private boolean swung;

    public void reset() {
        unwrapped = Double.NaN;
    }

    /// aim: any turn, deg. aimRate: deg/s. turretNow: deg, picks the starting turn. lead: s before the turret reacts.
    /// stopDecel: deg/s^2 it can count on to stop. margin: spare deg at the end
    public double update(double aim, double aimRate, double turretNow, double min, double max, double lead, double stopDecel, double margin) {

        if (Double.isNaN(unwrapped)) unwrapped = nearest(aim, turretNow, min, max);
        else unwrapped += Math.IEEEremainder(aim - unwrapped, 360);
        double mid = 0.5 * (min + max);
        boolean upper = unwrapped > mid;
        double outward = upper ? aimRate : -aimRate;
        double room = upper ? max - unwrapped : unwrapped - min;
        double stop = outward > 0 ? outward * lead + outward * outward / (2 * stopDecel) : 0;
        boolean racing = outward > 0 && room < stop + margin;
        boolean parked = room < -margin && -outward < -room - margin; //out past the end and not coming back in
        swung = racing || parked;
        if (swung) unwrapped += upper ? -360 : 360;
        return clamp(unwrapped, min, max);
    }

    /// waiting at an end for the aim to come round
    public boolean isOutside(double min, double max) {
        return unwrapped < min || unwrapped > max;
    }

    /// the last update swung a full turn
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
