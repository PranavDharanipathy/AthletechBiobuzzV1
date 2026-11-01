package org.firstinspires.ftc.teamcode.Utility;

/** Fault and recovery regression checks without SDK dependencies. */
public final class VoltageCompensatorTest {
    public static void main(String[] args) {
        VoltageCompensator c = new VoltageCompensator(0.05, 0.1, 12);
        equal(12, c.volts(), "nominal fallback");
        equal(0, c.toPower(Double.NaN), "NaN command stops");
        equal(0, c.toPower(Double.POSITIVE_INFINITY), "infinite command stops");
        equal(1, c.toPower(100), "positive saturation");
        equal(-1, c.toPower(-100), "negative saturation");
        c.addSample(12, 0);
        c.addSample(10, Double.NaN);
        check(c.shouldSample(0.1), "bad timestamp must not disable future sampling");
        c.addSample(10, -1);
        equal(12, c.volts(), "backwards time rejected");
        c.addSample(Double.NaN, 0.05);
        c.addSample(16, 0.1);
        c.addSample(10, 0.2);
        equal(12 + (1 - Math.exp(-2)) * -2, c.volts(), "filter recovers across missing readings");
        check(c.rejectedSamples() == 4, "rejections counted");
        check(!c.shouldSample(Double.POSITIVE_INFINITY), "invalid clock not sampled");
        VoltageCompensator unfiltered = new VoltageCompensator(0, 0, 12);
        unfiltered.addSample(11, 0);
        unfiltered.addSample(10, 0);
        equal(10, unfiltered.volts(), "zero tau bypasses filter even at equal time");
        invalid(-1, 0.1, 12);
        invalid(0.1, Double.NaN, 12);
        invalid(0.1, 0.1, 0);
        invalid(Double.POSITIVE_INFINITY, 0.1, 12);
        System.out.println("VoltageCompensator: fault, recovery and configuration checks passed");
    }

    private static void invalid(double period, double tau, double nominal) {
        try {
            new VoltageCompensator(period, tau, nominal);
        } catch (IllegalArgumentException expected) {
            return;
        }
        throw new AssertionError("invalid configuration accepted");
    }

    private static void equal(double expected, double actual, String message) {
        check(Double.isFinite(actual) && Math.abs(expected - actual) < 1e-12, message);
    }

    private static void check(boolean valid, String message) {
        if (!valid) throw new AssertionError(message);
    }
}
