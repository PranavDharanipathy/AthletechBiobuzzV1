package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * The complete shooter "brain" without hardware: aim → turret / flywheel / hood set-points → fire gate
 * (design D-9). Pure Java, so the full loop runs in simulation and unit tests. The hardware subsystem feeds
 * it sensor values and writes its outputs.
 */
public final class ShooterCore {

    public enum Mode { OFF, TRACK }

    /** Why a shot is (not) allowed. The first failing condition is reported (driver feedback, logs). */
    public enum Readiness {
        READY,
        OFF,
        SENSOR_FAULT,
        NO_SOLUTION,          // out of range / no valid trajectory
        TURRET_UNREACHABLE,   // target outside turret limits
        TURRET_AIMING,        // pointing error > tolerance
        TURRET_RATE,          // still moving relative to the target rate
        FLYWHEEL_SPEED,       // exit speed outside the compensable range
        FLYWHEEL_UNSTABLE,    // accelerating (just shot / spinning up)
        HOOD_MOVING,
        RECOVERING            // post-shot lockout
    }

    public static final class Config {
        public double wheelRadiusM = 0.036;            // 72 mm wheel
        /** Ball exit speed / wheel surface speed ❓ (calibrated with Shot Calibration). */
        public double exitEfficiency = 0.47;
        public double maxFlywheelSpeed = 580;          // rad/s usable (≈5540 rpm) at nominal battery
        public double idleFlywheelSpeed = 0;           // rad/s while tracking without a solution
        /** Flywheel acceleration magnitude (rad/s²) above which the speed is "unstable" for firing. */
        public double flywheelStableAccel = 150;
        public double hoodToleranceDeg = 0.6;
        public double turretRateToleranceDeg = 6;     // deg/s
        public double postShotLockout = 0.0;           // s (0 = rely on the readiness checks only)
        /** Residual launch-angle correction vs distance from calibration (deg), optional. */
        public double[] residualDistanceM = {0.5, 5.0};
        public double[] residualAngleDeg = {0.0, 0.0};
    }

    private final Config cfg;
    public final FlywheelController flywheel;
    public final TurretController turret;
    public final HoodModel hood;
    public final AimSolver aimSolver;
    public final HiveTarget target;
    private ShotTable table;
    private final InterpTable residual;

    private final AimSolver.Aim aim = new AimSolver.Aim();
    private Mode mode = Mode.OFF;
    private Readiness readiness = Readiness.OFF;
    private double flywheelVolts;
    private double hoodAngleCommand = Double.NaN;
    private boolean hoodWriteNeeded;
    private double now;
    private double lastFireTime = Double.NEGATIVE_INFINITY;
    private double prevFlywheelSpeed = Double.NaN, flywheelAccel;
    private double lastAimedFlywheelSpeed;

    public ShooterCore(Config cfg, FlywheelController flywheel, TurretController turret, HoodModel hood,
                       AimSolver aimSolver, HiveTarget target, ShotTable table) {
        this.cfg = cfg;
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.aimSolver = aimSolver;
        this.target = target;
        this.table = table;
        this.residual = new InterpTable(cfg.residualDistanceM, cfg.residualAngleDeg);
    }

    public void setMode(Mode m) { mode = m; }
    public Mode mode() { return mode; }
    public void setShotTable(ShotTable t) { table = t; }

    public double exitSpeedFor(double flywheelRadPerSec) {
        return cfg.exitEfficiency * cfg.wheelRadiusM * flywheelRadPerSec;
    }

    public double flywheelSpeedFor(double exitSpeed) {
        return exitSpeed / (cfg.exitEfficiency * cfg.wheelRadiusM);
    }

    /**
     * One loop.
     *
     * @param robot          latest localization
     * @param flywheelPosRad flywheel encoder position (rad), NaN on fault
     * @param turretAngleRad turret angle (rad, continuous), NaN on fault
     * @param batteryVolts   filtered battery voltage
     * @param t              monotonic time (s)
     */
    public void update(RobotState robot, double flywheelPosRad, double turretAngleRad, double batteryVolts, double t) {
        double dt = t - now;
        now = t;

        if (mode == Mode.OFF) {
            flywheel.setTarget(0);
            turret.disable();
            lastAimedFlywheelSpeed = 0;
        }

        boolean haveAim = false;
        if (mode == Mode.TRACK) {
            aimSolver.solve(robot, target, table, t, aim);
            haveAim = aim.valid;
            if (haveAim) {
                turret.setTarget(aim.turretAngle, aim.turretRate);
                double w = Math.min(cfg.maxFlywheelSpeed, flywheelSpeedFor(aim.shot.exitSpeed));
                flywheel.setTarget(w);
                lastAimedFlywheelSpeed = w;
            } else {
                // Momentarily out of range / bad pose: HOLD the last aimed speed (a re-spin-up costs ~1.2 s).
                flywheel.setTarget(Math.max(cfg.idleFlywheelSpeed, lastAimedFlywheelSpeed));
            }
        }

        flywheelVolts = flywheel.update(flywheelPosRad, t, batteryVolts);
        turret.update(turretAngleRad, t);

        double w = flywheel.velocity();
        if (dt > 0 && !Double.isNaN(prevFlywheelSpeed)) {
            flywheelAccel += MathUtil.lowPassAlpha(dt, 0.03) * ((w - prevFlywheelSpeed) / dt - flywheelAccel);
        }
        prevFlywheelSpeed = w;

        // Speed-adaptive hood: the angle that hits with the speed the wheel actually has.
        hoodWriteNeeded = false;
        double compAngle = Double.NaN;
        if (haveAim) {
            compAngle = table.compensatedAngle(aim.distanceM, exitSpeedFor(w));
            double base = Double.isNaN(compAngle) ? aim.shot.launchAngle : compAngle;
            double cmd = base + Math.toRadians(residual.get(aim.distanceM));
            hoodWriteNeeded = hood.command(cmd);
        }
        hoodAngleCommand = hood.commandedAngle();
        hood.update(t);

        readiness = evaluate(haveAim, compAngle);
    }

    private Readiness evaluate(boolean haveAim, double compAngle) {
        if (mode == Mode.OFF) return Readiness.OFF;
        if (flywheel.isFaulted() || turret.isFaulted()) return Readiness.SENSOR_FAULT;
        if (!haveAim) return Readiness.NO_SOLUTION;
        if (!turret.isTargetReachable()) return Readiness.TURRET_UNREACHABLE;
        double pointErr = Math.abs(MathUtil.wrapPi(aim.turretAngle - turret.angle()));
        if (pointErr > aim.tolerance) return Readiness.TURRET_AIMING;
        if (Math.abs(turret.rateError()) > Math.toRadians(cfg.turretRateToleranceDeg)) return Readiness.TURRET_RATE;
        if (Double.isNaN(compAngle)) return Readiness.FLYWHEEL_SPEED;
        if (Math.abs(flywheelAccel) > cfg.flywheelStableAccel) return Readiness.FLYWHEEL_UNSTABLE;
        if (!hood.isSettled(Math.toRadians(cfg.hoodToleranceDeg))) return Readiness.HOOD_MOVING;
        if (now - lastFireTime < cfg.postShotLockout) return Readiness.RECOVERING;
        return Readiness.READY;
    }

    /** Call when the feeder releases a ball. */
    public void notifyFired() {
        lastFireTime = now;
        flywheel.notifyFeed();
    }

    public boolean canFire() { return readiness == Readiness.READY; }
    public Readiness readiness() { return readiness; }
    public double flywheelVolts() { return flywheelVolts; }
    public double turretLeftPower() { return turret.leftPower(); }
    public double turretRightPower() { return turret.rightPower(); }
    /** Hood servo position for the current command (NaN until the first command). */
    public double hoodServoPosition() {
        return Double.isNaN(hoodAngleCommand) ? Double.NaN : hood.servoPositionFor(hoodAngleCommand);
    }
    public boolean hoodWriteNeeded() { return hoodWriteNeeded; }
    public AimSolver.Aim aim() { return aim; }
    public double flywheelAccel() { return flywheelAccel; }
    public Config config() { return cfg; }
}
