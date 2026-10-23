package org.firstinspires.ftc.teamcode.Utility;

public class TickrateChecker {

    private static double lastTime;
    private static double currentTime;

    /// Also does the calculation
    public static double getTimePerTick() {

        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
        return currentTime - lastTime;
    }

}