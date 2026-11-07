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
            //sum += (dx * y) + (0.5 * dx * (lastY - y)); (this is simplified into the line below)
            sum += 0.5 * (lastY + y) * dx; //adding area of trapezoid
        }

        lastY = y;
    }

    public void setSum(double value) {

        //a reset also starts a new integration interval, even if the total is unchanged
        //otherwise the next update integrates against a sample from before the reset
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
