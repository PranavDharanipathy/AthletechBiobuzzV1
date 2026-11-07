package org.firstinspires.ftc.teamcode.Utility;

import java.util.function.DoubleSupplier;

//robot time in seconds, the Sim swaps the source so robot code runs on sim time
public final class Clock {

    private static DoubleSupplier source = null;

    private Clock() {}

    public static double seconds() {
        return source == null ? System.nanoTime() * 1e-9 : source.getAsDouble();
    }

    /// null goes back to the real clock
    public static void useSource(DoubleSupplier simSource) {
        source = simSource;
    }
}
