package org.firstinspires.ftc.teamcode.shooter.sim;

/**
 * "True" flywheel physics for off-robot testing (deliberately richer than the controller's model):
 * DC motor (R, Kt, Ke), viscous + Coulomb friction, battery with internal resistance and an extra load
 * (drivetrain), PWM voltage = power × terminal voltage, integer 28-count encoder, ball contact energy loss.
 */
public final class FlywheelSim {
    // goBILDA 5202 6000 rpm priors (datasheet) with inertia ≈ CAD estimate
    public double R = 1.30, Kt = 0.0157, Ke = 0.0186, J = 2.0e-4;
    public double viscous = 4e-6;          // N·m/(rad/s)
    public double coulomb = 0.004;         // N·m
    public double countsPerRev = 28;

    public double batteryOpenCircuit = 12.8;
    public double batteryResistance = 0.06;   // Ω (pack + wiring)
    public double otherLoadCurrent = 0;       // A (e.g. drivetrain), can change during a test

    public double omega = 0, theta = 0, current = 0, terminalVolts = 12.8;
    private double power = 0;

    // ball contact
    private double contactEnd = -1, contactTorque = 0;

    public void setPower(double p) { power = Math.max(-1, Math.min(1, p)); }

    /** Starts a ball contact that removes energy dE (J) over duration (s). */
    public void ballContact(double t, double dE, double duration) {
        double e = 0.5 * J * omega * omega;
        double newOmega = Math.sqrt(Math.max(0, e - dE) * 2 / J);
        contactTorque = J * (omega - newOmega) / duration;
        contactEnd = t + duration;
    }

    public void step(double t, double dt) {
        // battery terminal voltage depends on total current (solve with the previous motor current)
        terminalVolts = batteryOpenCircuit - batteryResistance * (Math.abs(current) + otherLoadCurrent);
        double v = power * terminalVolts;
        current = (v - Ke * omega) / R;
        double torque = Kt * current - viscous * omega - coulomb * Math.signum(omega);
        if (t < contactEnd) torque -= contactTorque;
        omega += torque / J * dt;
        if (omega < 0 && power >= 0 && Math.abs(omega) < 1) omega = 0; // friction holds at rest
        theta += omega * dt;
    }

    /** Integer encoder count converted to radians, as the robot would see it. */
    public double measuredPositionRad() {
        double counts = Math.floor(theta / (2 * Math.PI) * countsPerRev);
        return counts * 2 * Math.PI / countsPerRev;
    }
}
