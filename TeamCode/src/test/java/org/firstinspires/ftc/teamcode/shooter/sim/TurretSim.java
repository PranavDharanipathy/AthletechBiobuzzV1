package org.firstinspires.ftc.teamcode.shooter.sim;

/**
 * "True" turret physics: two CR servos (deadband, linear torque–speed curve), each pinion meshing the ring
 * with backlash (stiff, damped contact), ring friction, and the inertial torque from robot yaw acceleration.
 * All angles are turret-referred (pinion angle / 3), in radians.
 */
public final class TurretSim {
    public double deadband = 0.03;            // servo command deadband (power)
    public double freeSpeed = 3.0;            // turret rad/s at power 1 (≈172°/s)
    public double stallTorque = 1.2;          // turret-referred N·m per servo (effective, incl. servo current limit)
    public double pinionInertia = 4e-4;       // turret-referred kg·m² per servo
    public double ringInertia = 8e-3;         // turret + shooter kg·m²
    public double ringCoulomb = 0.05, ringViscous = 0.02;
    public double pinionCoulombPower = 0.01;
    public double backlash = Math.toRadians(1.0);   // total lash at each mesh (turret degrees)
    public double contactStiffness = 300, contactDamping = 1.5;
    public double encoderCountsPerRev = 24576;      // REV Through Bore at 3:1 ❓

    public double ring, ringRate;
    public final double[] pin = new double[2], pinRate = new double[2];
    private final double[] power = new double[2];
    public double robotYawAccel = 0;

    public void setPowers(double left, double right) {
        power[0] = Math.max(-1, Math.min(1, left));
        power[1] = Math.max(-1, Math.min(1, right));
    }

    public void reset(double angle) {
        ring = angle;
        ringRate = 0;
        for (int i = 0; i < 2; i++) { pin[i] = angle; pinRate[i] = 0; }
    }

    public void step(double dt) {
        double ringTorque = 0;
        for (int i = 0; i < 2; i++) {
            double u = power[i];
            double ue = Math.abs(u) < deadband ? 0 : u - Math.signum(u) * deadband;
            ue /= (1 - deadband);
            double servoTorque = stallTorque * (ue - pinRate[i] / freeSpeed);
            if (Math.abs(pinRate[i]) < 1e-3 && Math.abs(ue) < pinionCoulombPower) servoTorque = 0;
            double rel = pin[i] - ring;
            double half = backlash / 2;
            double contact = 0;
            if (rel > half) contact = contactStiffness * (rel - half) + contactDamping * (pinRate[i] - ringRate);
            else if (rel < -half) contact = contactStiffness * (rel + half) + contactDamping * (pinRate[i] - ringRate);
            if ((rel > half && contact < 0) || (rel < -half && contact > 0)) contact = 0; // no pulling
            pinRate[i] += (servoTorque - contact) / pinionInertia * dt;
            pin[i] += pinRate[i] * dt;
            ringTorque += contact;
        }
        double fric = ringViscous * ringRate;
        if (Math.abs(ringRate) > 1e-4) fric += ringCoulomb * Math.signum(ringRate);
        else if (Math.abs(ringTorque) < ringCoulomb) { ringTorque = 0; ringRate = 0; }
        // relative to the robot: an accelerating robot drags the ring backwards (inertia)
        double acc = (ringTorque - fric) / ringInertia - robotYawAccel;
        ringRate += acc * dt;
        ring += ringRate * dt;
    }

    public double measuredAngle() {
        double counts = Math.floor(ring / (2 * Math.PI) * encoderCountsPerRev);
        return counts * 2 * Math.PI / encoderCountsPerRev;
    }
}
