package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

import java.util.List;

public class ShotSolver {

    public static final double GRAVITY = 386.09;

    private final FlywheelVelocityModel model;
    public double time;
    public int iterations;

    public ShotSolver(FlywheelVelocityModel model) {
        this.model = model;
    }

    public static double timeOfFlight(double d) {

        if (!ShooterConstants.SOTM_ENABLED || !(d > 0)) return 0;
        double t = measured(d);
        if (!(t > 0)) {
            double rise = d * Math.tan(Math.toRadians(ShooterConstants.SHOT_LAUNCH_ANGLE_DEGREES)) - ShooterConstants.SHOT_RISE_INCHES;
            t = rise > 0 ? Math.sqrt(2 * rise / GRAVITY) : 0;
        }
        return t * ShooterConstants.SOTM_TIME_SCALE;
    }

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
