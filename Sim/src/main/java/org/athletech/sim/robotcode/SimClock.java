package org.athletech.sim.robotcode;

import org.firstinspires.ftc.teamcode.Utility.Clock;

public final class SimClock implements AutoCloseable {

    private double seconds;

    public SimClock() {
        Clock.useSource(() -> seconds);
    }

    public void set(double seconds) {
        this.seconds = seconds;
    }

    public double get() {
        return seconds;
    }

    @Override
    public void close() {
        Clock.useSource(null);
    }
}
