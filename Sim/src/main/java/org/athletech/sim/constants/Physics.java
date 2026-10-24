package org.athletech.sim.constants;

/** Physical constants and unit conversions. Internally every model uses SI units and radians. */
public final class Physics {
    private Physics() {
    }

    /** Standard gravity, m/s^2. */
    public static final double G = 9.80665;
    /** Air density at 20 C, sea level, kg/m^3. */
    public static final double AIR_DENSITY = 1.204;
    /** Vacuum permittivity, F/m. */
    public static final double EPSILON_0 = 8.8541878128e-12;
    /** Smallest voltage that can spark across any air gap (Paschen minimum, air), V. */
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

    /** Wraps an angle to (-pi, pi]. */
    public static double wrap(double rad) {
        double a = Math.IEEEremainder(rad, 2 * Math.PI);
        return a <= -Math.PI ? a + 2 * Math.PI : a;
    }
}
