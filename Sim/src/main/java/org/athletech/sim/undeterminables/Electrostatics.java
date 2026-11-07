package org.athletech.sim.undeterminables;

import org.athletech.sim.constants.Physics;

public final class Electrostatics {
    private Electrostatics() {
    }

    public static double generationFactor(double rh, double scalePct) {
        return Math.exp(-(rh - 40) / Math.max(1e-6, scalePct));
    }

    public static double resistanceFactor(double rh, double decadesPer10Rh) {
        return Math.pow(10, -(rh - 40) / 10.0 * decadesPer10Rh);
    }

    public static double robotCapacitance(double lengthM, double widthM, double clearanceM, double areaFraction, double tileThicknessM, double tilePermittivity) {

        double area = lengthM * widthM * areaFraction;
        double gap = clearanceM + tileThicknessM / tilePermittivity;
        double plate = Physics.EPSILON_0 * area / gap;
        double self = 0.6607 * 4 * Math.PI * Physics.EPSILON_0 * Math.max(lengthM, widthM);
        return plate + self;
    }

    public static double inducedFraction(double clearanceM, double tileThicknessM, double tilePermittivity) {

        double down = tileThicknessM / tilePermittivity;
        return down / (down + clearanceM);
    }

    public static double contactEnergy(double c1, double c2, double dv) {
        return 0.5 * c1 * c2 / (c1 + c2) * dv * dv;
    }
}
