package org.firstinspires.ftc.teamcode.Utility;


public class InterpolationData {

    /// First indexed value is the data pair, the second is x or y of the data pair.
    /// <p>
    /// |0| |1| is the y value of the first data pair.
    public double[][] dataPoints;

    /**
     * @param dataPoint1 minimum
     * @param dataPoint2 maximum
     **/
    public InterpolationData(double[] dataPoint1, double[] dataPoint2) {

        dataPoints = new double[][] {dataPoint1, dataPoint2};
    }
}