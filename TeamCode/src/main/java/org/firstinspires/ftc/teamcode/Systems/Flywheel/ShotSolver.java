package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

import java.util.List;

/// shoot on the move, flywheel included. The robot's velocity only adds to the ball's horizontal velocity: how high the ball
/// climbs, and so how long it flies, comes from the flywheel alone (fixed launch angle, the robot doesn't move up). So a ball
/// launched at the flywheel speed of a still shot to distance D flies T(D) and covers D relative to the turret, plus v T from
/// the robot. It scores when D aimed along goal - v T reaches the goal, i.e. D = |g - v T(D)| (g = turret to goal).
///  - equivalentDistance: that D, the still-robot distance whose flywheel speed a moving shot needs (slower driving at the
///    goal, faster driving away or across)
///  - missInches: with the speed the flywheel is ACTUALLY at, how far a ball fired now would land long (+) or short (-)
///    along the aim; the blocker only opens while it's small, so a lagging or dipping flywheel can't throw a miss
/// Inches, seconds, ticks/s; g and v in the field frame. Pure Java, the Sim runs it
public class ShotSolver {

    public static final double GRAVITY = 386.09;

    private final FlywheelVelocityModel model;
    /// flight time of the last solve (s)
    public double time;
    public int iterations;

    public ShotSolver(FlywheelVelocityModel model) {
        this.model = model;
    }

    /// still-robot flight time to distance d: SHOT_TIMES_OF_FLIGHT when it's filled (measured), else a projectile at
    /// SHOT_LAUNCH_ANGLE_DEGREES climbing SHOT_RISE_INCHES; times SOTM_TIME_SCALE. 0 = no shoot on the move
    public static double timeOfFlight(double d) {

        if (!ShooterConstants.SOTM_ENABLED || !(d > 0)) return 0;
        double t = measured(d);
        if (!(t > 0)) {
            double rise = d * Math.tan(Math.toRadians(ShooterConstants.SHOT_LAUNCH_ANGLE_DEGREES)) - ShooterConstants.SHOT_RISE_INCHES;
            t = rise > 0 ? Math.sqrt(2 * rise / GRAVITY) : 0;
        }
        return t * ShooterConstants.SOTM_TIME_SCALE;
    }

    //linear between the measured points, held at the ends; 0 when none is filled
    private static double measured(double d) {

        List<Double> ds = ShooterConstants.FLYWHEEL_DISTANCES, ts = ShooterConstants.SHOT_TIMES_OF_FLIGHT;
        int n = Math.min(ds.size(), ts.size());
        boolean any = false;
        for (int i = 0; i < n; i++) any |= ts.get(i) > 0;
        if (!any) return 0;
        if (d <= ds.get(0)) return ts.get(0);
        if (d >= ds.get(n - 1)) return ts.get(n - 1);
        int i = 1;
        while (ds.get(i) < d) i++;
        double s = (d - ds.get(i - 1)) / (ds.get(i) - ds.get(i - 1));
        return ts.get(i - 1) + s * (ts.get(i) - ts.get(i - 1));
    }

    /// the still-robot distance whose flywheel speed this moving shot needs: the root of |g - v T(D)| - D, which is positive at
    /// D = 0 and negative once D is big (T only grows like sqrt D). Bracketed, then bisected: plain iteration oscillates when
    /// driving fast at a close goal. Sets time
    public double equivalentDistance(double gx, double gy, double vx, double vy) {

        iterations = 0;
        if (vx == 0 && vy == 0) {
            double d = Math.hypot(gx, gy);
            time = timeOfFlight(d);
            return d;
        }
        double lo = 0, hi = Math.max(1, Math.hypot(gx, gy));
        while (gap(gx, gy, vx, vy, hi) > 0 && iterations < 40) {
            lo = hi;
            hi *= 2;
            iterations++;
        }
        for (int k = 0; k < 60 && hi - lo > 1e-4; k++, iterations++) {
            double mid = 0.5 * (lo + hi);
            if (gap(gx, gy, vx, vy, mid) > 0) lo = mid;
            else hi = mid;
        }
        double d = 0.5 * (lo + hi);
        time = timeOfFlight(d);
        return d;
    }

    private static double gap(double gx, double gy, double vx, double vy, double d) {

        double t = timeOfFlight(d);
        return Math.hypot(gx - vx * t, gy - vy * t) - d;
    }

    /// a ball fired now at this (actual) flywheel speed, aimed along g - v T: how far it lands long (+) or short (-) of the
    /// goal along the aim (in). NaN if the speed is outside what the fit covers. Sets time (the aim should use it)
    public double missInches(double gx, double gy, double vx, double vy, double flywheelVelocity, double extrapolate) {

        double d = model.distanceFor(flywheelVelocity, extrapolate);
        if (d != d) {
            time = Double.NaN;
            return Double.NaN;
        }
        time = timeOfFlight(d);
        return d - Math.hypot(gx - vx * time, gy - vy * time);
    }
}
