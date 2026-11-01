package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Hood: launch angle ↔ servo position through the (nonlinear) four-bar linkage, plus a motion model of the
 * positional servos so we know when the hood has <i>actually</i> arrived (design D-4 / D-9).
 *
 * <p>The mapping is a measured table (Hood Calibration OpMode): launch angle (rad, from horizontal) vs servo
 * position [0..1]. The servo itself is open loop from our side, so its motion is predicted with a
 * rate-limited first-order model (servo slew rate + small settling lag).
 */
public final class HoodModel {

    public static final class Config {
        /**
         * Calibration points: servo positions and the measured launch angles (deg). Must be monotonic.
         * PLACEHOLDER ❓ until Hood Calibration is run. The range 40–80° is what the ballistics study says is
         * needed to reach the CELL from 0.7–3.5 m with ≥ 25° descent (see docs/SHOOTER_DESIGN.md §6).
         */
        public double[] servoPositions = {0.10, 0.30, 0.50, 0.70, 0.90};
        public double[] launchAnglesDeg = {80, 70, 60, 50, 40};
        /** Hood angular slew rate when moving (launch-angle deg/s). Axon Max ≈ 520°/s at the horn; linkage ❓. */
        public double slewRateDegPerSec = 250;
        /** Additional first-order settling time constant (s). */
        public double settleTau = 0.03;
        /** Commands closer than this (deg) to the last one are not re-sent (servo jitter, I/O saving). */
        public double commandDeadbandDeg = 0.05;
        /** Mirrored second servo: position = mirrorOffset − position (1.0 for a symmetric mount) ❓. */
        public double mirrorOffset = 1.0;
    }

    private final Config cfg;
    private final InterpTable angleToServo;
    private final InterpTable servoToAngle;
    private final double minAngle, maxAngle;

    private double commandedAngle = Double.NaN;  // rad
    private double estimatedAngle = Double.NaN;  // rad (model of where the hood actually is)
    private double lagAngle = Double.NaN;        // intermediate (slew-limited) state
    private double now;
    private boolean haveTime;

    public HoodModel(Config cfg) {
        this.cfg = cfg;
        double[] anglesRad = new double[cfg.launchAnglesDeg.length];
        for (int i = 0; i < anglesRad.length; i++) anglesRad[i] = Math.toRadians(cfg.launchAnglesDeg[i]);
        servoToAngle = InterpTable.fromUnsorted(cfg.servoPositions, anglesRad);
        angleToServo = servoToAngle.inverse();
        minAngle = angleToServo.minX();
        maxAngle = angleToServo.maxX();
    }

    public double minAngle() { return minAngle; }
    public double maxAngle() { return maxAngle; }

    /** Servo position for a launch angle (clamped to the calibrated range). */
    public double servoPositionFor(double launchAngleRad) {
        return angleToServo.get(MathUtil.clamp(launchAngleRad, minAngle, maxAngle));
    }

    public double angleForServoPosition(double pos) {
        return servoToAngle.get(pos);
    }

    public double mirroredPosition(double pos) {
        return MathUtil.clamp(cfg.mirrorOffset - pos, 0, 1);
    }

    /**
     * Commands a new launch angle. Returns true if the servo should actually be written (i.e. the change is
     * larger than the deadband). The caller writes {@link #servoPositionFor(double)} of {@link #commandedAngle()}.
     */
    public boolean command(double launchAngleRad) {
        if (!MathUtil.isFinite(launchAngleRad)) return false;
        double a = MathUtil.clamp(launchAngleRad, minAngle, maxAngle);
        if (!Double.isNaN(commandedAngle) && Math.abs(a - commandedAngle) < Math.toRadians(cfg.commandDeadbandDeg)) {
            return false;
        }
        commandedAngle = a;
        if (Double.isNaN(estimatedAngle)) {
            // unknown start: assume it will take a full-range move
            estimatedAngle = a > (minAngle + maxAngle) / 2 ? minAngle : maxAngle;
            lagAngle = estimatedAngle;
        }
        return true;
    }

    /** Advances the motion model. */
    public void update(double t) {
        double dt = haveTime ? t - now : 0;
        now = t;
        haveTime = true;
        if (Double.isNaN(commandedAngle) || !(dt > 0)) return;
        double slew = Math.toRadians(cfg.slewRateDegPerSec) * dt;
        lagAngle = MathUtil.approach(lagAngle, commandedAngle, slew);
        estimatedAngle += MathUtil.lowPassAlpha(dt, cfg.settleTau) * (lagAngle - estimatedAngle);
    }

    /** True if the hood is predicted to be within tol (rad) of the commanded angle. */
    public boolean isSettled(double tolRad) {
        return !Double.isNaN(commandedAngle) && Math.abs(estimatedAngle - commandedAngle) <= tolRad;
    }

    /** Predicted time (s) to reach the commanded angle from the present estimate. */
    public double timeToArrive() {
        if (Double.isNaN(commandedAngle)) return Double.POSITIVE_INFINITY;
        return Math.abs(commandedAngle - lagAngle) / Math.toRadians(cfg.slewRateDegPerSec) + 3 * cfg.settleTau;
    }

    public double commandedAngle() { return commandedAngle; }
    public double estimatedAngle() { return estimatedAngle; }
    public Config config() { return cfg; }
}
