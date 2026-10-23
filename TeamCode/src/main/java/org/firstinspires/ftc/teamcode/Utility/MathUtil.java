package org.firstinspires.ftc.teamcode.Utility;

import org.apache.commons.math3.util.FastMath;

import java.math.BigDecimal;
import java.util.Arrays;
import java.util.List;

public strictfp class MathUtil {

    private MathUtil() {}

    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }

    public static long clamp(long value, long min, long max) {
        return Math.max(min, Math.min(value, max));
    }

    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static Integer toInteger(int intValue) {
        return intValue;
    }

    public static Long toLong(long longValue) {
        return longValue;
    }

    public static Float toFloat(float floatValue) {
        return floatValue;
    }

    public static Double toDouble(double doubleValue) {
        return doubleValue;
    }

    public static double nanosecondsToMilliseconds(double nanoseconds) {
        return nanoseconds / 1_000_000.0;
    }

    public static double millisecondsToNanoseconds(double milliseconds) {
        return milliseconds * 1_000_000.0;
    }

    public static double secondsToNanoseconds(double seconds) {
        return seconds * 1_000_000_000.0;
    }

    public static BigDecimal secondsToNanosecondsSafe(double seconds) {
        return new BigDecimal(seconds).multiply(new BigDecimal("1000000000.0"));
    }

    public static double nanosecondsToSeconds(double nanoseconds) {
        return nanoseconds / 1_000_000_000.0;
    }

    public static double secondsToMilliseconds(double seconds) {
        return seconds * 1_000.0;
    }

    public static double millisecondsToSeconds(double milliseconds) {
        return milliseconds / 1_000.0;
    }

    public static double getDistance2d(double x1, double x2, double y1, double y2) {

        double deltaX = x2 - x1;
        double deltaY = y2 - y1;

        return FastMath.sqrt((deltaX * deltaX) + (deltaY * deltaY));
    }

    public static double getDistance3d(double x1, double x2, double y1, double y2, double z1, double z2) {

        double deltaX = x2 - x1;
        double deltaY = y2 - y1;
        double deltaZ = z2 - z1;

        return FastMath.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    }

    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    public static double metersToCentimeters(double meters) {
        return meters * 100d;
    }
    public static double metersToMillimeters(double meters) {
        return meters * 1000d;
    }
    public static double inchesToCentimeters(double inches) {
        return inches * 2.54;
    }
    public static double inchesToMillimeters(double inches) {
        return inches * 25.4;
    }
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    public static double centimetersToInches(double meters) {
        return meters * 0.393701;
    }

    public static double centimetersToMillimeters(double meters) {
        return meters * 10;
    }
    public static double centimetersToMeters(double meters) {
        return meters * 0.01;
    }
    public static double millimetersToInches(double meters) {
        return meters * 0.0393701;
    }

    public static double millimetersToCentimeters(double meters) {
        return meters * 0.1;
    }
    public static double millimetersToMeters(double meters) {
        return meters * 0.001;
    }

    public static double deadband(double targetValue, double currentValue, double deadbandValue) {

        double deltaValue = targetValue - currentValue;

        return Math.abs(deltaValue) < deadbandValue ? currentValue : targetValue;
    }

    public static double truncate(double value, double truncateFactor) {
        return Math.round(value * truncateFactor) / truncateFactor;
    }

    public static boolean valueWithinRange(double value, double minRange, double maxRange) {
        return value > minRange && value < maxRange;
    }

    public static boolean valueWithinRangeIncludingPoles(double value, double minRange, double maxRange) {
        return value >= minRange && value <= maxRange;
    }

    /// @param array must be sorted
    public static double[] findBoundingValues(double[] array, double value) {

        for (int index = 0; index < array.length - 1; index++) {
            double lower = array[index];
            double upper = array[index + 1];

            if (value >= lower && value <= upper) {
                return new double[] {lower, upper};
            }
        }

        throw new IllegalArgumentException("No bounding values for input: value cannot be found from input array!");
    }

    public static double[] sortAscending(double[] values) {

        double[] sorted = Arrays.copyOf(values, values.length);
        Arrays.sort(sorted);
        return sorted;
    }

    public static double interpolateLinear(double x, InterpolationData data) {

        double x0 = data.dataPoints[0][0];
        double y0 = data.dataPoints[0][1];

        double x1 = data.dataPoints[1][0];
        double y1 = data.dataPoints[1][1];

        return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
    }

    public static double normalizeAngleRad(double angleRad) {
        return FastMath.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    public static double normalizeAngleDeg(double angleDeg) {
        return Math.toDegrees(FastMath.atan2(Math.sin(Math.toRadians(angleDeg)), Math.cos(Math.toRadians(angleDeg))));
    }

    /// @param theta - how much the coordinates are being rotated in radians
    public static double[] rotateCartesian(double x, double y, double theta) {

        double xNew = x * Math.cos(theta) - y * Math.sin(theta);
        double yNew = x * Math.sin(theta) + y * Math.cos(theta);

        return new double[] {xNew, yNew};
    }

    public static double increaseMagnitudeNumerical(double value, double increment) {
        return value >= 0 ? value + increment : value - increment;
    }

    public static int getClosestIndex(List<Double> values, double skibidiValue) {

        if (values == null || values.isEmpty()) {
            throw new IllegalArgumentException("List cannot be null or empty");
        }

        int closestIndex = 0;
        double smallestDiff = Math.abs(values.get(0) - skibidiValue);

        for (int i = 1; i < values.size(); i++) {
            double diff = Math.abs(values.get(i) - skibidiValue);

            if (diff < smallestDiff) {
                smallestDiff = diff;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    public static double getMean(List<Double> values) {

        if (values == null || values.isEmpty()) {
            throw new IllegalArgumentException("List cannot be null or empty");
        }

        double sum = 0.0;

        for (double value : values) {
            sum += value;
        }

        return sum / values.size();
    }
}
