package org.athletech.sim.physics;

import java.util.function.DoubleUnaryOperator;

/**
 * Battery as an open-circuit voltage behind an internal resistance (battery + wiring + switch).
 * Terminal voltage sags with total current: {@code V = Voc - R*I}. The open-circuit voltage drops
 * slowly as charge is used.
 */
public final class Battery {
    private final double vocFull;
    private final double resistance;
    private final double vocDropPerAh;
    private double usedAh;
    private double lastVolts;
    private double lastCurrent;

    /**
     * @param openCircuitVolts resting voltage at the start
     * @param internalOhms     total series resistance
     * @param vocDropPerAh     how much the resting voltage falls per amp-hour used (0 = ideal)
     */
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

    /**
     * Finds the terminal voltage V where {@code V = Voc - R * totalCurrent(V)}.
     * {@code totalCurrent} returns the battery current all loads draw at bus voltage V.
     */
    public double solve(DoubleUnaryOperator totalCurrent) {
        double voc = openCircuitVolts();
        double v = lastVolts > 0 ? lastVolts : voc;
        for (int k = 0; k < 40; k++) {
            double next = Math.max(0, voc - resistance * totalCurrent.applyAsDouble(v));
            if (Math.abs(next - v) < 1e-7) {
                v = next;
                break;
            }
            v = 0.5 * (v + next); // damped fixed point; converges while R * dI/dV < 3 (it is ~0.2 on FTC robots)
        }
        lastVolts = v;
        lastCurrent = resistance > 0 ? (voc - v) / resistance : totalCurrent.applyAsDouble(v);
        return v;
    }

    /** Integrates charge drawn over {@code dt} seconds at the last solved current. */
    public void drain(double dt) {
        usedAh += Math.max(0, lastCurrent) * dt / 3600.0;
    }
}
