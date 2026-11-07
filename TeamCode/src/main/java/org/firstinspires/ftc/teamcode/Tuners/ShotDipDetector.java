package org.firstinspires.ftc.teamcode.Tuners;

public class ShotDipDetector {

    private static final int CAPACITY = 256;

    private final double windowSeconds;
    private double threshold;

    private final double[] times = new double[CAPACITY];
    private final double[] velocities = new double[CAPACITY];
    private int head = 0, size = 0;

    private boolean armed = true;
    private double dipMinimum = Double.POSITIVE_INFINITY;
    private double contactVelocity = Double.NaN;

    public ShotDipDetector(double windowSeconds, double threshold) {

        this.windowSeconds = windowSeconds;
        this.threshold = threshold;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public double getThreshold() {
        return threshold;
    }

    public boolean update(double now, double velocity, boolean feeding) {

        if (Double.isNaN(velocity)) return false;

        times[head] = now;
        velocities[head] = velocity;
        head = (head + 1) % CAPACITY;
        if (size < CAPACITY) size++;

        if (!armed) {
            dipMinimum = Math.min(dipMinimum, velocity);
            if (velocity >= dipMinimum + threshold / 2) {
                armed = true;
                size = 1;
            }
            return false;
        }

        double peak = Double.NEGATIVE_INFINITY;
        for (int k = 0; k < size; k++) {
            int index = (head - 1 - k + CAPACITY) % CAPACITY;
            if (now - times[index] > windowSeconds) break;
            peak = Math.max(peak, velocities[index]);
        }

        if (feeding && peak - velocity >= threshold) {
            armed = false;
            dipMinimum = velocity;
            contactVelocity = peak;
            return true;
        }
        return false;
    }

    public double getContactVelocity() {
        return contactVelocity;
    }
}
