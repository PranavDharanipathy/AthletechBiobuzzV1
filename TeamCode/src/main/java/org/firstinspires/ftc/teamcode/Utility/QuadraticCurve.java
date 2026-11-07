package org.firstinspires.ftc.teamcode.Utility;

/// stick to output: x^2 from (0,0) to (1,1), same sign as the stick, so everything under full stick goes slower and finer
public class QuadraticCurve {

    public double getOutput(double x) {

        double size = Math.min(Math.abs(x), 1);
        return Math.signum(x) * size * size;
    }
}
