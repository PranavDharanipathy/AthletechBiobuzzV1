package org.firstinspires.ftc.teamcode.Utility;

import java.util.Locale;

//tick() once per loop; telemetry is only sent ~4 times a second, so read what the loop measured
public class TickRateChecker {

    public static double WORST_WINDOW_SECONDS = 5;

    private double last = Double.NaN, timePerTick = Double.NaN, average = Double.NaN, worst = 0, windowStart = Double.NaN;

    public void tick() {

        double now = Clock.seconds();
        if (!Double.isNaN(last)) {
            timePerTick = 1000 * (now - last);
            average = Double.isNaN(average) ? timePerTick : average + 0.05 * (timePerTick - average);
            if (Double.isNaN(windowStart) || now - windowStart > WORST_WINDOW_SECONDS) {
                worst = timePerTick;
                windowStart = now;
            }
            else worst = Math.max(worst, timePerTick);
        }
        last = now;
    }

    public double getTimePerTick() {
        return timePerTick;
    }

    public double getAverage() {
        return average;
    }

    public double getWorst() {
        return worst;
    }

    public String describe() {

        if (Double.isNaN(average)) return "-";
        return String.format(Locale.US, "%.1f ms (avg %.1f ms, %.0f Hz, worst %.0f ms)", timePerTick, average, 1000 / average, worst);
    }
}
