package org.athletech.sim.predict;

import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.MecanumDrive;

import java.util.function.Supplier;

/**
 * Physics-limited move times: full power toward the goal, then full reverse power to stop exactly
 * on it ("bang-bang", the fastest any controller could do). The switch point is found by
 * bisection. Real path followers are slower; scale by {@code drive.path_overhead_factor}.
 */
public final class DriveTimePredictor {
    private DriveTimePredictor() {
    }

    public enum Axis { FORWARD, STRAFE, TURN }

    public static final double DT = 0.002;

    /** Result of one stop-to-stop move. */
    public static final class Move {
        private final double timeS;
        private final double peakSpeed;
        private final double peakCurrentA;
        private final double minBusV;

        public Move(double timeS, double peakSpeed, double peakCurrentA, double minBusV) {
            this.timeS = timeS;
            this.peakSpeed = peakSpeed;
            this.peakCurrentA = peakCurrentA;
            this.minBusV = minBusV;
        }

        public double timeS() { return timeS; }
        public double peakSpeed() { return peakSpeed; }
        public double peakCurrentA() { return peakCurrentA; }
        public double minBusV() { return minBusV; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Move)) return false;
            Move o = (Move) other;
            return Double.compare(timeS, o.timeS) == 0
                    && Double.compare(peakSpeed, o.peakSpeed) == 0
                    && Double.compare(peakCurrentA, o.peakCurrentA) == 0
                    && Double.compare(minBusV, o.minBusV) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(timeS, peakSpeed, peakCurrentA, minBusV);
        }

        @Override
        public String toString() {
            return "Move[timeS=" + timeS + ", peakSpeed=" + peakSpeed + ", peakCurrentA=" + peakCurrentA + ", minBusV=" + minBusV + "]";
        }
    }

    /**
     * @param amount  meters for FORWARD/STRAFE, radians for TURN (positive)
     * @param battery makes a fresh battery for each trial run
     */
    public static Move timeTo(MecanumDrive.Params p, Supplier<Battery> battery, double otherCurrentA, Axis axis, double amount) {
        if (amount <= 0) return new Move(0, 0, 0, Double.NaN);
        double lo = 0, hi = amount;
        Move best = null;
        double bestErr = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 32; i++) {
            double sw = 0.5 * (lo + hi);
            double[] out = new double[1];
            Move m = run(p, battery.get(), otherCurrentA, axis, sw, out);
            if (out[0] < amount) lo = sw;
            else hi = sw;
            double err = Math.abs(out[0] - amount);
            if (err < bestErr) {
                bestErr = err;
                best = m;
            }
            if (err < amount * 1e-4) break;
        }
        return best;
    }

    /** Accelerate until {@code switchAt}, then reverse until stopped. out[0] = where it stopped. */
    private static Move run(MecanumDrive.Params p, Battery battery, double otherCurrentA, Axis axis, double switchAt, double[] out) {
        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        double t = 0, peakV = 0, peakI = 0, minV = Double.POSITIVE_INFINITY;
        boolean braking = false;
        while (t < 30) {
            double pos = positionOf(d, axis), vel = velocityOf(d, axis);
            if (!braking && pos >= switchAt) braking = true;
            if (braking && vel <= 0) break;
            double cmd = braking ? -1 : 1;
            switch (axis) {
                case FORWARD:
                    d.drive(cmd, 0, 0);
                    break;
                case STRAFE:
                    d.drive(0, cmd, 0);
                    break;
                case TURN:
                    d.drive(0, 0, cmd);
                    break;
            }
            d.step(DT, battery, otherCurrentA);
            t += DT;
            peakV = Math.max(peakV, Math.abs(velocityOf(d, axis)));
            peakI = Math.max(peakI, d.motorCurrentTotal());
            minV = Math.min(minV, d.busVolts());
        }
        out[0] = positionOf(d, axis);
        return new Move(t, peakV, peakI, minV);
    }

    public static double positionOf(MecanumDrive d, Axis axis) {
        switch (axis) {
            case FORWARD: return d.x();
            case STRAFE: return d.y();
            default: return d.heading();
        }
    }

    public static double velocityOf(MecanumDrive d, Axis axis) {
        switch (axis) {
            case FORWARD: return d.forwardSpeed();
            case STRAFE: return d.leftSpeed();
            default: return d.omega();
        }
    }
}
