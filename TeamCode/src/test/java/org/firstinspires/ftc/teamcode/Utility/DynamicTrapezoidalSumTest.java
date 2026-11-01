package org.firstinspires.ftc.teamcode.Utility;

/** Plain Java regression checks; run with tools/run_utility_tests.sh. */
public final class DynamicTrapezoidalSumTest {
    public static void main(String[] args) {
        DynamicTrapezoidalSum sum = new DynamicTrapezoidalSum();
        sum.updateSum(1, 10);
        sum.setSum(0);
        sum.updateSum(1, 2);
        expect(0, sum.getSum(), "zero reset discards the previous sample");
        sum.updateSum(2, 4);
        expect(6, sum.getSum(), "integration resumes from the new sample");

        sum.setSum(6);
        sum.updateSum(1, 100);
        expect(6, sum.getSum(), "unchanged nonzero reset discards the previous sample");

        sum.setRawSum(3);
        sum.updateSum(1, 102);
        expect(104, sum.getSum(), "raw anti-windup clamp preserves sample history");
        System.out.println("DynamicTrapezoidalSum: 4 regression checks passed");
    }

    private static void expect(double expected, double actual, String message) {
        if (actual != expected) {
            throw new AssertionError(message + ": expected " + expected + ", got " + actual);
        }
    }
}
