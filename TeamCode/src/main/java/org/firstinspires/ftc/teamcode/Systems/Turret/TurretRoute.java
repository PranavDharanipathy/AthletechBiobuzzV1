package org.firstinspires.ftc.teamcode.Systems.Turret;

public class TurretRoute {

    private double unwrapped = Double.NaN, outsideSince = Double.NaN, lastSwing = Double.NEGATIVE_INFINITY, lastAim = Double.NaN;
    private boolean swung;

    public void reset() {

        unwrapped = Double.NaN;
        outsideSince = Double.NaN;
        lastSwing = Double.NEGATIVE_INFINITY;
        lastAim = Double.NaN;
    }

    public double update(double aim, double aimRate, double turretNow, double min, double max, double margin, double now) {

        //a relocalization teleports the aim: following it round from the old side ran to the far end then swung a full turn
        boolean jumped = Math.abs(Math.IEEEremainder(aim - lastAim, 360)) > JUMP_DEGREES;
        lastAim = aim;
        if (Double.isNaN(unwrapped) || jumped) {
            unwrapped = nearest(aim, turretNow, min, max);
            outsideSince = Double.NaN;
        }
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

    public static final double RETURN_SECONDS = 1, SWING_DELAY = 0.25, SWING_COOLDOWN = 1, FAR_PAST = 45, JUMP_DEGREES = 90;

    public boolean isOutside(double min, double max) {
        return unwrapped < min || unwrapped > max;
    }

    public boolean hasSwung() {
        return swung;
    }

    //the aim's turn nearest the turret is within 180 of it; if that's past an end the next turn in is the closest legal one
    private static double nearest(double aim, double turretNow, double min, double max) {

        double c = turretNow + Math.IEEEremainder(aim - turretNow, 360);
        if (c > max) c -= 360;
        else if (c < min) c += 360;
        if (c >= min && c <= max) return c;
        return aim - 360 * Math.floor((aim - min) / 360);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
