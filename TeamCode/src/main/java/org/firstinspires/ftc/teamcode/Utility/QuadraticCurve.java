package org.firstinspires.ftc.teamcode.Utility;

public class QuadraticCurve {

    public double getOutput(double x) {

        double size = Math.min(Math.abs(x), 1);
        return Math.signum(x) * size * size;
    }
}
