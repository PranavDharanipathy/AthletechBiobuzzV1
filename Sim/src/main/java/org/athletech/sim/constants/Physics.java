package org.athletech.sim.constants;

public final class Physics {
    private Physics() {
    }

    public static final double G = 9.80665;
    public static final double AIR_DENSITY = 1.204;
    public static final double EPSILON_0 = 8.8541878128e-12;
    public static final double PASCHEN_MIN_V = 327;

    public static final double INCH = 0.0254;
    public static final double KG_CM_TO_NM = 0.0980665;
    public static final double OZ_IN_TO_NM = 0.00706155;

    public static double rpmToRadPerSec(double rpm) {
        return rpm * 2 * Math.PI / 60.0;
    }

    public static double radPerSecToRpm(double w) {
        return w * 60.0 / (2 * Math.PI);
    }

    public static double deg(double rad) {
        return Math.toDegrees(rad);
    }

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }

    public static double wrap(double rad) {

        double a = Math.IEEEremainder(rad, 2 * Math.PI);
        return a <= -Math.PI ? a + 2 * Math.PI : a;
    }
}
