package org.athletech.sim.physics;

import java.util.function.DoubleUnaryOperator;

public final class Battery {
    private final double vocFull;
    private final double resistance;
    private final double vocDropPerAh;
    private double usedAh;
    private double lastVolts;
    private double lastCurrent;

    public Battery(double openCircuitVolts, double internalOhms, double vocDropPerAh) {

        this.vocFull = openCircuitVolts;
        this.resistance = Math.max(0, internalOhms);
        this.vocDropPerAh = Math.max(0, vocDropPerAh);
        this.lastVolts = openCircuitVolts;
    }

    public double openCircuitVolts() {
        return vocFull - vocDropPerAh * usedAh;
    }

    public double resistance() { return resistance; }
    public double lastVolts() { return lastVolts; }
    public double lastCurrent() { return lastCurrent; }
    public double usedMah() { return usedAh * 1000; }

    public double solve(DoubleUnaryOperator totalCurrent) {

        double voc = openCircuitVolts();
        double v = lastVolts > 0 ? lastVolts : voc;
        for (int k = 0; k < 40; k++) {
            double next = Math.max(0, voc - resistance * totalCurrent.applyAsDouble(v));
            if (Math.abs(next - v) < 1e-7) {
                v = next;
                break;
            }
            v = 0.5 * (v + next);
        }
        lastVolts = v;
        lastCurrent = resistance > 0 ? (voc - v) / resistance : totalCurrent.applyAsDouble(v);
        return v;
    }

    public void drain(double dt) {
        usedAh += Math.max(0, lastCurrent) * dt / 3600.0;
    }
}
