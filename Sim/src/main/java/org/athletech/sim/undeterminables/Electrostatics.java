package org.athletech.sim.undeterminables;

import org.athletech.sim.constants.Physics;

/**
 * Electrostatic helpers: how humidity changes charging and leakage, the robot's capacitance from
 * its geometry, and how much of the tiles' surface charge a robot "sees".
 *
 * <p>Humidity: a thin water film on plastics and foam conducts. Surface resistivity falls roughly
 * a decade per few % RH, so leakage resistances scale as {@code 10^(-(RH-40)/10 * decades)}.
 * Charging (triboelectric separation) also falls with humidity, modelled as
 * {@code exp(-(RH-40)/scale)}. Both are relative to 40% RH, where the profile's numbers are given.
 */
public final class Electrostatics {
    private Electrostatics() {
    }

    /** Charging multiplier at {@code rh} % relative to 40% RH. */
    public static double generationFactor(double rh, double scalePct) {
        return Math.exp(-(rh - 40) / Math.max(1e-6, scalePct));
    }

    /** Leakage resistance multiplier at {@code rh} % relative to 40% RH (drier = higher). */
    public static double resistanceFactor(double rh, double decadesPer10Rh) {
        return Math.pow(10, -(rh - 40) / 10.0 * decadesPer10Rh);
    }

    /**
     * Frame capacitance to the floor (F): the metal facing the tiles acts as a plate over the floor
     * (air gap plus tile thickness / permittivity), plus the free-space self-capacitance of a box the
     * robot's size (a cube of side a has C = 0.6607 * 4 pi eps0 * a).
     */
    public static double robotCapacitance(double lengthM, double widthM, double clearanceM, double areaFraction,
                                          double tileThicknessM, double tilePermittivity) {
        double area = lengthM * widthM * areaFraction;
        double gap = clearanceM + tileThicknessM / tilePermittivity;
        double plate = Physics.EPSILON_0 * area / gap;
        double self = 0.6607 * 4 * Math.PI * Physics.EPSILON_0 * Math.max(lengthM, widthM);
        return plate + self;
    }

    /**
     * Share of a tile surface charge's field that ends on a conductor plate above it rather than the
     * floor below: the charge sheet sits between the floor (tile thickness / permittivity away) and
     * the frame (clearance away), and its field lines split in inverse proportion to those distances.
     */
    public static double inducedFraction(double clearanceM, double tileThicknessM, double tilePermittivity) {
        double down = tileThicknessM / tilePermittivity;
        return down / (down + clearanceM);
    }

    /** Energy (J) released when two isolated conductors at dV touch: 1/2 * C1*C2/(C1+C2) * dV^2. */
    public static double contactEnergy(double c1, double c2, double dv) {
        return 0.5 * c1 * c2 / (c1 + c2) * dv * dv;
    }
}
