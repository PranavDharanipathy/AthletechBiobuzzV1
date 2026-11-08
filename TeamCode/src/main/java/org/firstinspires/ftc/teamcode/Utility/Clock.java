package org.firstinspires.ftc.teamcode.Utility;

import java.util.function.DoubleSupplier;

public final class Clock {

    private static DoubleSupplier source = null;

    private Clock() {}

    public static double seconds() {
        return source == null ? System.nanoTime() * 1e-9 : source.getAsDouble();
    }

    public static void useSource(DoubleSupplier simSource) {
        source = simSource;
    }
}
