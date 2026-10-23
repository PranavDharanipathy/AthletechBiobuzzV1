package org.athletech.sim.robot;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.DcMotor;
import org.athletech.sim.physics.MotorSpec;

/**
 * Mecanum drivetrain dynamics: four motors driven from a sagging battery, traction-limited wheel
 * forces, rolling resistance, and strafing losses. Robot frame: +x forward, +y left, yaw CCW.
 *
 * <p>Wheel order FL, FR, BL, BR, rollers in the usual X pattern seen from above. Wheel surface
 * speeds and forces follow the standard mecanum Jacobian:
 * <pre>
 *   wheel speed FL = vx - vy/eta - k*w     forces:  Fx = fFL + fFR + fBL + fBR
 *               FR = vx + vy/eta + k*w              Fy = eta * (-fFL + fFR + fBL - fBR)
 *               BL = vx + vy/eta - k*w              Tz = k   * (-fFL + fFR - fBL + fBR)
 *               BR = vx - vy/eta + k*w              k  = (trackWidth + wheelbase) / 2
 * </pre>
 * {@code eta} (strafe efficiency) lumps roller slip/friction: strafing needs 1/eta more wheel
 * speed and gives eta of the force. Each wheel's force is capped by traction, mu*N/sqrt(2) (the
 * 45-degree rollers push diagonally). Simplification: wheels never spin faster than the chassis
 * allows (no wheel-slip dynamics); a capped wheel just delivers less force.
 */
public final class MecanumDrive {
    public static final int FL = 0, FR = 1, BL = 2, BR = 3;

    public static final class Params {
        private final double massKg;
        private final double moiKgm2;
        private final double wheelRadiusM;
        private final double trackWidthM;
        private final double wheelbaseM;
        private final double gearRatio;
        private final MotorSpec motor;
        private final double frictionCoeff;
        private final double strafeEfficiency;
        private final double rollingResistanceCoeff;
        private final double currentLimitA;
        private final double maxPower;
        private final boolean brake;

        public Params(double massKg, double moiKgm2, double wheelRadiusM, double trackWidthM, double wheelbaseM, double gearRatio, MotorSpec motor, double frictionCoeff, double strafeEfficiency, double rollingResistanceCoeff, double currentLimitA, double maxPower, boolean brake) {
            if (!(massKg > 0) || !(moiKgm2 > 0) || !(wheelRadiusM > 0) || !(gearRatio > 0)) {
                throw new IllegalArgumentException("mass, inertia, wheel radius and gear ratio must be positive");
            }
            if (!(strafeEfficiency > 0 && strafeEfficiency <= 1)) throw new IllegalArgumentException("strafe efficiency must be in (0, 1]");
        
            this.massKg = massKg;
            this.moiKgm2 = moiKgm2;
            this.wheelRadiusM = wheelRadiusM;
            this.trackWidthM = trackWidthM;
            this.wheelbaseM = wheelbaseM;
            this.gearRatio = gearRatio;
            this.motor = motor;
            this.frictionCoeff = frictionCoeff;
            this.strafeEfficiency = strafeEfficiency;
            this.rollingResistanceCoeff = rollingResistanceCoeff;
            this.currentLimitA = currentLimitA;
            this.maxPower = maxPower;
            this.brake = brake;
        }

        public double massKg() { return massKg; }
        public double moiKgm2() { return moiKgm2; }
        public double wheelRadiusM() { return wheelRadiusM; }
        public double trackWidthM() { return trackWidthM; }
        public double wheelbaseM() { return wheelbaseM; }
        public double gearRatio() { return gearRatio; }
        public MotorSpec motor() { return motor; }
        public double frictionCoeff() { return frictionCoeff; }
        public double strafeEfficiency() { return strafeEfficiency; }
        public double rollingResistanceCoeff() { return rollingResistanceCoeff; }
        public double currentLimitA() { return currentLimitA; }
        public double maxPower() { return maxPower; }
        public boolean brake() { return brake; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Params)) return false;
            Params o = (Params) other;
            return Double.compare(massKg, o.massKg) == 0
                    && Double.compare(moiKgm2, o.moiKgm2) == 0
                    && Double.compare(wheelRadiusM, o.wheelRadiusM) == 0
                    && Double.compare(trackWidthM, o.trackWidthM) == 0
                    && Double.compare(wheelbaseM, o.wheelbaseM) == 0
                    && Double.compare(gearRatio, o.gearRatio) == 0
                    && java.util.Objects.equals(motor, o.motor)
                    && Double.compare(frictionCoeff, o.frictionCoeff) == 0
                    && Double.compare(strafeEfficiency, o.strafeEfficiency) == 0
                    && Double.compare(rollingResistanceCoeff, o.rollingResistanceCoeff) == 0
                    && Double.compare(currentLimitA, o.currentLimitA) == 0
                    && Double.compare(maxPower, o.maxPower) == 0
                    && brake == o.brake;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(massKg, moiKgm2, wheelRadiusM, trackWidthM, wheelbaseM, gearRatio, motor, frictionCoeff, strafeEfficiency, rollingResistanceCoeff, currentLimitA, maxPower, brake);
        }

        @Override
        public String toString() {
            return "Params[massKg=" + massKg + ", moiKgm2=" + moiKgm2 + ", wheelRadiusM=" + wheelRadiusM + ", trackWidthM=" + trackWidthM + ", wheelbaseM=" + wheelbaseM + ", gearRatio=" + gearRatio + ", motor=" + motor + ", frictionCoeff=" + frictionCoeff + ", strafeEfficiency=" + strafeEfficiency + ", rollingResistanceCoeff=" + rollingResistanceCoeff + ", currentLimitA=" + currentLimitA + ", maxPower=" + maxPower + ", brake=" + brake + "]";
        }

        /** Free speed of the robot driving forward, ignoring every loss, m/s. */
        public double theoreticalTopSpeed() {
            return motor.freeSpeedRadPerSec() / gearRatio * wheelRadiusM;
        }
    
    }

    private final Params p;
    private final DcMotor[] motors = new DcMotor[4];
    private final double k;
    private final double tractionLimit;

    // field-frame state
    private double x, y, heading, vx, vy, omega;
    private final double[] wheelCurrent = new double[4];
    private final boolean[] slipping = new boolean[4];
    private double busVolts, batteryCurrent;

    public MecanumDrive(Params p) {
        this.p = p;
        for (int i = 0; i < 4; i++) {
            motors[i] = new DcMotor(p.motor());
            motors[i].setBrake(p.brake());
            motors[i].setCurrentLimit(p.currentLimitA());
        }
        this.k = (p.trackWidthM() + p.wheelbaseM()) / 2;
        this.tractionLimit = p.frictionCoeff() > 0
                ? p.frictionCoeff() * p.massKg() * Physics.G / 4 / Math.sqrt(2)
                : Double.POSITIVE_INFINITY;
    }

    public Params params() { return p; }

    public void reset(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        vx = vy = omega = 0;
    }

    /** Raw wheel powers in [-1, 1], scaled by the configured max power. */
    public void setPowers(double fl, double fr, double bl, double br) {
        double cap = p.maxPower() > 0 ? p.maxPower() : 1;
        motors[FL].setDuty(fl * cap);
        motors[FR].setDuty(fr * cap);
        motors[BL].setDuty(bl * cap);
        motors[BR].setDuty(br * cap);
    }

    /** Robot-centric drive with the usual TeleOp mixing. Inputs in [-1, 1]. */
    public void drive(double forward, double strafeLeft, double turnCcw) {
        double fl = forward - strafeLeft - turnCcw;
        double fr = forward + strafeLeft + turnCcw;
        double bl = forward + strafeLeft - turnCcw;
        double br = forward - strafeLeft + turnCcw;
        double m = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        setPowers(fl / m, fr / m, bl / m, br / m);
    }

    /** Advances the simulation by {@code dt}. {@code otherBatteryCurrentA} = everything else on the battery. */
    public void step(double dt, Battery battery, double otherBatteryCurrentA) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxr = c * vx + s * vy, vyr = -s * vx + c * vy;
        double eta = p.strafeEfficiency();
        double[] surface = {
                vxr - vyr / eta - k * omega,
                vxr + vyr / eta + k * omega,
                vxr + vyr / eta - k * omega,
                vxr - vyr / eta + k * omega};
        double[] shaft = new double[4];
        for (int i = 0; i < 4; i++) shaft[i] = surface[i] / p.wheelRadiusM() * p.gearRatio();

        busVolts = battery.solve(v -> {
            double total = otherBatteryCurrentA;
            for (int i = 0; i < 4; i++) total += motors[i].batteryCurrent(motors[i].current(v, shaft[i]));
            return total;
        });
        batteryCurrent = battery.lastCurrent();

        double[] f = new double[4];
        for (int i = 0; i < 4; i++) {
            double cur = motors[i].current(busVolts, shaft[i]);
            wheelCurrent[i] = cur;
            double wheelTorque = motors[i].torque(cur, shaft[i]) * p.gearRatio();
            double force = wheelTorque / p.wheelRadiusM();
            slipping[i] = Math.abs(force) > tractionLimit;
            f[i] = slipping[i] ? Math.copySign(tractionLimit, force) : force;
        }
        double fx = f[FL] + f[FR] + f[BL] + f[BR];
        double fy = eta * (-f[FL] + f[FR] + f[BL] - f[BR]);
        double tz = k * (-f[FL] + f[FR] - f[BL] + f[BR]);

        // rolling resistance: opposes motion, never reverses it, holds the robot when the push is small
        double rr = p.rollingResistanceCoeff() * p.massKg() * Physics.G;
        double sp = Math.hypot(vxr, vyr);
        if (sp > 1e-4) {
            double cap = Math.min(rr, p.massKg() * sp / dt);
            fx -= cap * vxr / sp;
            fy -= cap * vyr / sp;
        } else {
            double fn = Math.hypot(fx, fy);
            if (fn <= rr) {
                fx = fy = 0;
            } else {
                fx -= rr * fx / fn;
                fy -= rr * fy / fn;
            }
        }
        double rrT = rr * k / 2;
        if (Math.abs(omega) > 1e-4) {
            tz -= Math.copySign(Math.min(rrT, p.moiKgm2() * Math.abs(omega) / dt), omega);
        } else if (Math.abs(tz) <= rrT) {
            tz = 0;
        } else {
            tz -= Math.copySign(rrT, tz);
        }

        double ax = (c * fx - s * fy) / p.massKg();
        double ay = (s * fx + c * fy) / p.massKg();
        vx += ax * dt;
        vy += ay * dt;
        omega += tz / p.moiKgm2() * dt;
        x += vx * dt;
        y += vy * dt;
        heading += omega * dt;
        battery.drain(dt);
    }

    // ------------------------------------------------------------------ state

    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return heading; }
    /** Field-frame velocity. */
    public double vx() { return vx; }
    public double vy() { return vy; }
    public double omega() { return omega; }

    /** Robot-frame velocity: forward and left components. */
    public double forwardSpeed() { return Math.cos(heading) * vx + Math.sin(heading) * vy; }
    public double leftSpeed() { return -Math.sin(heading) * vx + Math.cos(heading) * vy; }
    public double speed() { return Math.hypot(vx, vy); }

    public double busVolts() { return busVolts; }
    /** Total battery current of the last step (drive + other loads), A. */
    public double batteryCurrent() { return batteryCurrent; }

    /** Sum of |winding current| over the four drive motors, A. */
    public double motorCurrentTotal() {
        double t = 0;
        for (double c : wheelCurrent) t += Math.abs(c);
        return t;
    }

    public boolean anySlipping() {
        for (boolean b : slipping) if (b) return true;
        return false;
    }
}
