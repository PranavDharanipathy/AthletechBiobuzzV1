package org.firstinspires.ftc.teamcode.Utility;


public class InterpolationData {

    public double[][] dataPoints;

    public InterpolationData(double[] dataPoint1, double[] dataPoint2) {
        dataPoints = new double[][] {dataPoint1, dataPoint2};
    }
}