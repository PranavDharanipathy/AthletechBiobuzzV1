package org.firstinspires.ftc.teamcode.Utility;

public class DynamicTrapezoidalSum {

    private double sum = 0d;

    private double lastY;
    private boolean hasLastY;

    public void updateSum(double dx, double y) {

        if (!hasLastY) {

            lastY = y;
            hasLastY = true;
            return;
        }

        if (dx != 0) {
            sum += 0.5 * (lastY + y) * dx;
        }

        lastY = y;
    }

    public void setSum(double value) {

        sum = value;
        hasLastY = false;
    }

    public void setRawSum(double value) {

        if (sum == value) return;

        sum = value;
    }

    public double getSum() {
        return sum;
    }
}
