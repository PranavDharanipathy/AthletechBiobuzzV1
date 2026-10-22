package org.firstinspires.ftc.teamcode.shooter.core;

/** Small allocation-free math helpers. Pure Java (unit-testable off robot). */
public final class MathUtil {

    public static final double TAU = 2.0 * Math.PI;
    public static final double IN_TO_M = 0.0254;
    public static final double M_TO_IN = 1.0 / IN_TO_M;
    public static final double RPM_TO_RAD_S = TAU / 60.0;

    private MathUtil() {}

    public static double clamp(double x, double lo, double hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrapPi(double a) {
        a = a % TAU;
        if (a <= -Math.PI) a += TAU;
        else if (a > Math.PI) a -= TAU;
        return a;
    }

    /** Moves {@code current} toward {@code target} by at most {@code maxStep}. */
    public static double approach(double current, double target, double maxStep) {
        double d = target - current;
        if (d > maxStep) return current + maxStep;
        if (d < -maxStep) return current - maxStep;
        return target;
    }

    /**
     * Smooth sign: tanh(x / width). Avoids the discontinuity of sgn() at zero, which otherwise
     * makes friction feedforward chatter when the velocity estimate hovers around 0.
     */
    public static double smoothSign(double x, double width) {
        if (width <= 0) return Math.signum(x);
        double r = x / width;
        if (r > 4) return 1;
        if (r < -4) return -1;
        double e = Math.exp(2 * r);
        return (e - 1) / (e + 1);
    }

    public static boolean isFinite(double x) {
        return !Double.isNaN(x) && !Double.isInfinite(x);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /** First-order low-pass coefficient for a time constant tau and step dt. */
    public static double lowPassAlpha(double dt, double tau) {
        if (tau <= 0) return 1;
        return 1.0 - Math.exp(-dt / tau);
    }
}
