package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Robot pose and velocity in the field frame, supplied by localization (e.g. Pinpoint). Mutable and
 * reused, so there are no allocations in the loop.
 *
 * <p>Units: inches, radians (CCW positive), seconds. Velocities are field-frame.
 */
public final class RobotState {
    public double x, y, heading;
    public double vx, vy, omega;
    /** Time (s, same clock as the loop) at which this state was measured. */
    public double timestamp;

    public RobotState set(double x, double y, double heading, double vx, double vy, double omega, double t) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        this.timestamp = t;
        return this;
    }

    public boolean isFinite() {
        return MathUtil.isFinite(x) && MathUtil.isFinite(y) && MathUtil.isFinite(heading)
                && MathUtil.isFinite(vx) && MathUtil.isFinite(vy) && MathUtil.isFinite(omega);
    }
}
