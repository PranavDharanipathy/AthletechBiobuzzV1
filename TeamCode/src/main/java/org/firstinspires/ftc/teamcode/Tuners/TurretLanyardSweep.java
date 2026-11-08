package org.firstinspires.ftc.teamcode.Tuners;

public final class TurretLanyardSweep {

    private TurretLanyardSweep() {}

    public static double period(double low, double high, double speed, double dwell) {
        return 2 * (high - low) / speed + 2 * dwell;
    }

    public static double position(double t, double low, double high, double speed, double dwell) {

        double glide = (high - low) / speed, toTop = high / speed;
        double u = phase(t + (glide - toTop) + dwell, low, high, speed, dwell);
        if (u < dwell) return low;
        if (u < dwell + glide) return low + speed * (u - dwell);
        if (u < 2 * dwell + glide) return high;
        return high - speed * (u - 2 * dwell - glide);
    }

    public static double velocity(double t, double low, double high, double speed, double dwell) {

        double glide = (high - low) / speed, toTop = high / speed;
        double u = phase(t + (glide - toTop) + dwell, low, high, speed, dwell);
        if (u < dwell) return 0;
        if (u < dwell + glide) return speed;
        if (u < 2 * dwell + glide) return 0;
        return -speed;
    }

    private static double phase(double t, double low, double high, double speed, double dwell) {

        double p = period(low, high, speed, dwell);
        return ((t % p) + p) % p;
    }
}
