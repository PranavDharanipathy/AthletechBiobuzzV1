package org.athletech.sim.physics;

import org.athletech.sim.constants.Physics;

/**
 * Brushed DC motor datasheet values, measured at the <b>output shaft</b> (gearbox included), at
 * {@code nominalVolts}. Everything else is derived from the standard linear motor model:
 *
 * <pre>
 *   V = I*R + Kv*w          (applied voltage = resistive drop + back-EMF)
 *   T = Kt*I - T_friction   (shaft torque)
 * </pre>
 *
 * with {@code R = V/I_stall}, {@code Kt = T_stall/(I_stall - I_free)},
 * {@code Kv = (V - I_free*R)/w_free} and {@code T_friction = Kt*I_free}. These reproduce the
 * datasheet exactly: free speed at free current, stall torque at stall current.
 */
public final class MotorSpec {
    private final double stallTorqueNm;
    private final double stallCurrentA;
    private final double freeSpeedRadPerSec;
    private final double freeCurrentA;
    private final double nominalVolts;

    public MotorSpec(double stallTorqueNm, double stallCurrentA, double freeSpeedRadPerSec, double freeCurrentA, double nominalVolts) {
        if (!(stallTorqueNm > 0) || !(stallCurrentA > 0) || !(freeSpeedRadPerSec > 0) || !(nominalVolts > 0)) {
            throw new IllegalArgumentException("motor spec needs positive stall torque, stall current, free speed and voltage");
        }
        if (!(freeCurrentA >= 0) || freeCurrentA >= stallCurrentA) {
            throw new IllegalArgumentException("free current must be >= 0 and below stall current");
        }
    
        this.stallTorqueNm = stallTorqueNm;
        this.stallCurrentA = stallCurrentA;
        this.freeSpeedRadPerSec = freeSpeedRadPerSec;
        this.freeCurrentA = freeCurrentA;
        this.nominalVolts = nominalVolts;
    }

    public double stallTorqueNm() { return stallTorqueNm; }
    public double stallCurrentA() { return stallCurrentA; }
    public double freeSpeedRadPerSec() { return freeSpeedRadPerSec; }
    public double freeCurrentA() { return freeCurrentA; }
    public double nominalVolts() { return nominalVolts; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof MotorSpec)) return false;
        MotorSpec o = (MotorSpec) other;
        return Double.compare(stallTorqueNm, o.stallTorqueNm) == 0
                && Double.compare(stallCurrentA, o.stallCurrentA) == 0
                && Double.compare(freeSpeedRadPerSec, o.freeSpeedRadPerSec) == 0
                && Double.compare(freeCurrentA, o.freeCurrentA) == 0
                && Double.compare(nominalVolts, o.nominalVolts) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(stallTorqueNm, stallCurrentA, freeSpeedRadPerSec, freeCurrentA, nominalVolts);
    }

    @Override
    public String toString() {
        return "MotorSpec[stallTorqueNm=" + stallTorqueNm + ", stallCurrentA=" + stallCurrentA + ", freeSpeedRadPerSec=" + freeSpeedRadPerSec + ", freeCurrentA=" + freeCurrentA + ", nominalVolts=" + nominalVolts + "]";
    }

    public static MotorSpec fromRpm(double stallTorqueNm, double stallCurrentA, double freeSpeedRpm,
                                    double freeCurrentA, double nominalVolts) {
        return new MotorSpec(stallTorqueNm, stallCurrentA, Physics.rpmToRadPerSec(freeSpeedRpm), freeCurrentA, nominalVolts);
    }

    /** Winding (plus brush/lead) resistance, ohms. */
    public double resistance() {
        return nominalVolts / stallCurrentA;
    }

    /** Torque constant, N*m per amp (at the output shaft). */
    public double kt() {
        return stallTorqueNm / (stallCurrentA - freeCurrentA);
    }

    /** Back-EMF constant, volts per rad/s (at the output shaft). */
    public double kv() {
        return (nominalVolts - freeCurrentA * resistance()) / freeSpeedRadPerSec;
    }

    /** Gearbox + bearing friction, N*m, taken from the free current. */
    public double frictionTorque() {
        return kt() * freeCurrentA;
    }

    /** Same motor behind an extra reduction {@code ratio} (output speed / ratio, torque * ratio * efficiency). */
    public MotorSpec geared(double ratio, double efficiency) {
        return new MotorSpec(stallTorqueNm * ratio * efficiency, stallCurrentA, freeSpeedRadPerSec / ratio, freeCurrentA, nominalVolts);
    }
}
