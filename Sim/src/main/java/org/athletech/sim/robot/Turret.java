package org.athletech.sim.robot;

/**
 * Turret slew timing: accelerate at {@code maxAccelDps2} up to {@code maxSpeedDps}, decelerate,
 * then settle. Range-limited turrets that must go "the long way" are the caller's business.
 */
public final class Turret {
    private final double maxSpeedDps;
    private final double maxAccelDps2;
    private final double settleS;
    private final double rangeDeg;

    public Turret(double maxSpeedDps, double maxAccelDps2, double settleS, double rangeDeg) {
        if (!(maxSpeedDps > 0) || !(maxAccelDps2 > 0)) throw new IllegalArgumentException("turret speed and acceleration must be positive");
    
        this.maxSpeedDps = maxSpeedDps;
        this.maxAccelDps2 = maxAccelDps2;
        this.settleS = settleS;
        this.rangeDeg = rangeDeg;
    }

    public double maxSpeedDps() { return maxSpeedDps; }
    public double maxAccelDps2() { return maxAccelDps2; }
    public double settleS() { return settleS; }
    public double rangeDeg() { return rangeDeg; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Turret)) return false;
        Turret o = (Turret) other;
        return Double.compare(maxSpeedDps, o.maxSpeedDps) == 0
                && Double.compare(maxAccelDps2, o.maxAccelDps2) == 0
                && Double.compare(settleS, o.settleS) == 0
                && Double.compare(rangeDeg, o.rangeDeg) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(maxSpeedDps, maxAccelDps2, settleS, rangeDeg);
    }

    @Override
    public String toString() {
        return "Turret[maxSpeedDps=" + maxSpeedDps + ", maxAccelDps2=" + maxAccelDps2 + ", settleS=" + settleS + ", rangeDeg=" + rangeDeg + "]";
    }

    /** Seconds to rotate {@code deg} degrees and settle. */
    public double timeToTurn(double deg) {
        double d = Math.abs(deg);
        if (d == 0) return 0;
        double v = maxSpeedDps, a = maxAccelDps2;
        double move = d >= v * v / a ? d / v + v / a : 2 * Math.sqrt(d / a);
        return move + Math.max(0, settleS);
    }
}
