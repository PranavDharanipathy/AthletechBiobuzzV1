package org.firstinspires.ftc.teamcode.Systems.Flywheel;

//tools/field_geometry.py in Java (docs/FIELD.md): the CELL's pentagon opening leaning back 30 deg, a drag-free ball at the launch
//angle, and how much slower / faster than the speed that hits the aim point still goes in from where the turret is
public final class CellGeometry {

    public static final double G = 386.09, POLLEN_RADIUS = 1.4, TILT_DEGREES = 30, OPENING_BOTTOM_Z = 53.5, AIM_H = 7.61;
    private static final double[] PU = {-10, 10, 10, 0, -10}, PH = {0, 0, 7.61, 14, 7.61};
    private static final int STEPS = 30;

    //lo = 1 - slow, hi = 1 + fast, as ratios of aimSpeed (in/s); NaN = no way in from here
    public double slow = Double.NaN, fast = Double.NaN, approachDegrees = Double.NaN, aimSpeed = Double.NaN;

    private double tx, ty, z0, wx, wy, reach, tanA, cosA, out, by, bz, ny, nz, hy, hz;

    public static double aimZ() {
        return OPENING_BOTTOM_Z + AIM_H * Math.cos(Math.toRadians(TILT_DEGREES));
    }

    //goal = the aim point (FieldConstants); close CELLs open toward +y, far ones toward -y
    public boolean update(double goalX, double goalY, boolean close, double turretX, double turretY, double exitHeight, double launchDegrees) {

        slow = fast = aimSpeed = Double.NaN;
        double t = Math.toRadians(TILT_DEGREES), a = Math.toRadians(launchDegrees);
        out = close ? 1 : -1;
        ny = out * Math.cos(t);
        nz = Math.sin(t);
        hy = -out * Math.sin(t);
        hz = Math.cos(t);
        by = goalY + out * AIM_H * Math.sin(t);
        bz = OPENING_BOTTOM_Z;
        tx = turretX;
        ty = turretY;
        z0 = exitHeight;
        double dx = goalX - turretX, dy = goalY - turretY;
        reach = Math.hypot(dx, dy);
        approachDegrees = Math.toDegrees(Math.atan2(Math.abs(dx), Math.abs(dy)));
        if (!(reach > 1e-6) || !(Math.cos(a) > 1e-6)) return false;
        wx = dx / reach;
        wy = dy / reach;
        tanA = Math.tan(a);
        cosA = Math.cos(a);
        double rise = reach * tanA - (aimZ() - exitHeight);
        if (!(rise > 0)) return false;
        aimSpeed = Math.sqrt(G * reach * reach / (2 * cosA * cosA * rise));
        if (!goesIn(1)) return false;
        slow = 1 - edge(-1);
        fast = edge(1) - 1;
        return true;
    }

    private double edge(int direction) {

        double in = 1, outside = 1 + 0.4 * direction;
        for (int k = 0; k < STEPS; k++) {
            double mid = 0.5 * (in + outside);
            if (goesIn(mid)) in = mid;
            else outside = mid;
        }
        return in;
    }

    //where the ball first crosses the opening's plane coming from the front, clear of every edge by its radius
    private boolean goesIn(double ratio) {

        double vh = aimSpeed * ratio * cosA;
        double a2 = -nz * G / (2 * vh * vh), a1 = ny * wy + nz * tanA, a0 = ny * (ty - by) + nz * (z0 - bz);
        //the later root: from low down the arc first rises out through the face's plane below the CELL, then comes back in
        double disc = a1 * a1 - 4 * a2 * a0;
        if (disc < 0) return false;
        double s = (-a1 - Math.sqrt(disc)) / (2 * a2);
        if (!(s > 0) || s > reach + 40) return false;
        double z = z0 + s * tanA - G * s * s / (2 * vh * vh);
        if (z < 0) return false;
        double u = tx + wx * s - (tx + wx * reach);
        double dyb = ty + wy * s - by, dzb = z - bz;
        double h = hy * dyb + hz * dzb;
        return inside(u, h, POLLEN_RADIUS);
    }

    static boolean inside(double u, double h, double margin) {

        for (int k = 0; k < PU.length; k++) {
            int n = (k + 1) % PU.length;
            double ex = PU[n] - PU[k], ey = PH[n] - PH[k];
            if (((u - PU[k]) * -ey + (h - PH[k]) * ex) / Math.hypot(ex, ey) < margin) return false;
        }
        return true;
    }
}
