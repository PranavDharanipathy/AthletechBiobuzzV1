package org.athletech.sim.physics;

/**
 * One motor on a REV hub port: duty-cycle command, optional current limit, BRAKE or FLOAT at zero
 * power. Stateless apart from the command; the mechanism that owns it integrates speed.
 */
public final class DcMotor {
    private final MotorSpec spec;
    private final double r, kt, kv, friction;
    private double duty;
    private double currentLimitA = Double.POSITIVE_INFINITY;
    private boolean brake = true;

    public DcMotor(MotorSpec spec) {
        this.spec = spec;
        this.r = spec.resistance();
        this.kt = spec.kt();
        this.kv = spec.kv();
        this.friction = spec.frictionTorque();
    }

    public MotorSpec spec() { return spec; }

    /** Power command in [-1, 1] (like {@code DcMotor.setPower}). */
    public void setDuty(double duty) {
        this.duty = Math.max(-1, Math.min(1, duty));
    }

    public double duty() { return duty; }

    public void setCurrentLimit(double amps) {
        this.currentLimitA = amps > 0 ? amps : Double.POSITIVE_INFINITY;
    }

    public void setBrake(boolean brake) { this.brake = brake; }

    /**
     * Winding current for this bus voltage and shaft speed (rad/s at the output shaft). Positive
     * current drives forward. With duty 0 a braking motor shorts its windings (current opposes
     * motion); a floating motor carries no current.
     */
    public double current(double busVolts, double omega) {
        if (duty == 0 && !brake) return 0;
        double i = (duty * busVolts - kv * omega) / r;
        return Math.max(-currentLimitA, Math.min(currentLimitA, i));
    }

    /** Current drawn from the battery for a given winding current (PWM: duty * I). */
    public double batteryCurrent(double windingCurrent) {
        return duty * windingCurrent;
    }

    /**
     * Shaft speed (rad/s) at which this motor's shaft torque equals {@code torque}, searching from
     * {@code from} in direction {@code dir} (+1 or -1), the way the torque falls. A wheel that slips
     * spins up (or skids down) until the motor's torque drops to what traction can hold; wheel
     * inertia is small, so it gets there within milliseconds. Exact on the straight part of the
     * motor curve; bisection where the current limit or friction makes it piecewise.
     */
    public double speedForTorque(double busVolts, double torque, double from, double dir) {
        if (kv > 0 && kt > 0 && r > 0) {
            for (int sg = -1; sg <= 1; sg += 2) {
                double w = (duty * busVolts - r * (torque + friction * sg) / kt) / kv;
                if (Math.abs(w) > 1e-9 && Math.signum(w) != sg) continue;
                if ((w - from) * dir < -1e-9) continue;
                if (Math.abs((duty * busVolts - kv * w) / r) > currentLimitA) continue;
                if (duty == 0 && !brake) continue;
                return w;
            }
        }
        double step = Math.max(1, spec.freeSpeedRadPerSec()), lo = from, hi = from + dir * step;
        for (int g = 0; g < 30 && dir * (torque(current(busVolts, hi), hi) - torque) > 0; g++) {
            lo = hi;
            hi += dir * step;
        }
        for (int k = 0; k < 60; k++) {
            double mid = 0.5 * (lo + hi);
            if (dir * (torque(current(busVolts, mid), mid) - torque) > 0) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    /**
     * Shaft torque for a winding current at shaft speed {@code omega}. Friction opposes motion;
     * at standstill it holds up to its value (static friction).
     */
    public double torque(double windingCurrent, double omega) {
        double t = kt * windingCurrent;
        if (Math.abs(omega) > 1e-6) return t - Math.copySign(friction, omega);
        if (Math.abs(t) <= friction) return 0;
        return t - Math.copySign(friction, t);
    }
}
