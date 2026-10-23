package org.firstinspires.ftc.teamcode.Utility;

public class DynamicTrapezoidalSum {

    private double sum = 0d;

    private Double lastY = null;

    public void updateSum(double dx, double y) {

        if (lastY == null) {

            lastY = y;
            return;
        }

        if (dx != 0) {
            //sum += (dx * y) + (0.5 * dx * (lastY - y)); (this is simplified into the line below)
            sum += 0.5 * (lastY + y) * dx; //adding area of trapezoid
        }

        lastY = y;
    }

    public void setSum(double value) {

        if (sum == value) return;

        sum = value;
        lastY = null;
    }

    public void setRawSum(double value) {

        if (sum == value) return;

        sum = value;
    }

    public double getSum() {
        return sum;
    }
}
