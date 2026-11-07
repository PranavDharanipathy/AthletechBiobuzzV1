package org.firstinspires.ftc.teamcode.Utility;

public class FastSigmoidCurve {

    public double W, H, A, B, S;

    public FastSigmoidCurve(double width, double height, double xOffset, double yOffset, double steepness) {

        W = width;
        H = height;
        A = xOffset;
        B = yOffset;
        S = steepness;
    }

    public FastSigmoidCurve() { //default curve
        this (2, 1, 1, 0.5, 1);
    }

    public void set(double width, double height, double xOffset, double yOffset, double steepness) {

        W = width;
        H = height;
        A = xOffset;
        B = yOffset;
        S = steepness;
    }

    public double getOutput(double x) {

        final double z = W * x - A;

        return H * z / (Math.abs(z) + S) + B;
    }
}
