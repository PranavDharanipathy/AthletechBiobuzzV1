package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Identified actuator model: u = kS*sgn(v) + kV*v + kA*a.
 *
 * <p>For the flywheel, u is in volts and v in rad/s. For the turret (servos), u is the servo power command in
 * [-1, 1] and v in turret rad/s. The values come from the characterization OpModes, not from guessing.
 */
public final class MotorFeedforward {

    public final double kS;
    public final double kV;
    public final double kA;
    /** Velocity width of the smooth sign used for the static-friction term (same units as v). */
    public final double signWidth;

    public MotorFeedforward(double kS, double kV, double kA, double signWidth) {
        if (!(kV > 0) || !(kA > 0) || kS < 0) {
            throw new IllegalArgumentException("need kV>0, kA>0, kS>=0");
        }
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.signWidth = signWidth;
    }

    /** Input needed to hold velocity v while accelerating at a. */
    public double calculate(double v, double a) {
        return kS * MathUtil.smoothSign(v, signWidth) + kV * v + kA * a;
    }

    /** Acceleration the model predicts for input u at velocity v (used by the observers). */
    public double acceleration(double u, double v) {
        return (u - kS * MathUtil.smoothSign(v, signWidth) - kV * v) / kA;
    }

    /** Largest acceleration reachable at velocity v with input magnitude uMax (>= 0). */
    public double maxAcceleration(double v, double uMax) {
        return Math.max(0.0, (uMax - kS * MathUtil.smoothSign(v, signWidth) - kV * v) / kA);
    }

    /** Free-running speed reachable with input uMax. */
    public double maxVelocity(double uMax) {
        return Math.max(0.0, (uMax - kS) / kV);
    }
}
