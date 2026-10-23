package org.athletech.sim.robot;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.DcMotor;
import org.athletech.sim.physics.MotorSpec;

/**
 * Flywheel launcher: motors (geared to the wheel) spin an inertia against friction under a
 * feedforward + proportional velocity controller, powered from a sagging battery. Firing a ball
 * takes kinetic energy out of the wheel, so back-to-back shots leave the wheel slower and the
 * controller needs time (and current) to recover.
 *
 * <p>Ball exit speed = exitSpeedRatio * wheel surface speed during contact. Energy taken from the
 * wheel = energyLossFactor * (1/2 m v^2) of the ball, where the factor (>= 1) covers ball spin,
 * compression and slip.
 */
public final class Flywheel {

    /**
     * @param gearRatio            motor shaft speed / flywheel speed (0.5 = flywheel spins twice as fast)
     * @param kpPerRpm             controller gain: duty added per rpm of error
     * @param voltageCompensation  scale the feedforward by 12 V / battery (like a voltage-compensated kF)
     */
    public static final class Params {
        private final double moiKgm2;
        private final double radiusM;
        private final double gearRatio;
        private final int motorCount;
        private final MotorSpec motor;
        private final double frictionTorqueNm;
        private final double kpPerRpm;
        private final boolean voltageCompensation;

        public Params(double moiKgm2, double radiusM, double gearRatio, int motorCount, MotorSpec motor, double frictionTorqueNm, double kpPerRpm, boolean voltageCompensation) {
            if (!(moiKgm2 > 0) || !(radiusM > 0) || !(gearRatio > 0) || motorCount < 1) {
                throw new IllegalArgumentException("flywheel inertia, radius, gear ratio and motor count must be positive");
            }
        
            this.moiKgm2 = moiKgm2;
            this.radiusM = radiusM;
            this.gearRatio = gearRatio;
            this.motorCount = motorCount;
            this.motor = motor;
            this.frictionTorqueNm = frictionTorqueNm;
            this.kpPerRpm = kpPerRpm;
            this.voltageCompensation = voltageCompensation;
        }

        public double moiKgm2() { return moiKgm2; }
        public double radiusM() { return radiusM; }
        public double gearRatio() { return gearRatio; }
        public int motorCount() { return motorCount; }
        public MotorSpec motor() { return motor; }
        public double frictionTorqueNm() { return frictionTorqueNm; }
        public double kpPerRpm() { return kpPerRpm; }
        public boolean voltageCompensation() { return voltageCompensation; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Params)) return false;
            Params o = (Params) other;
            return Double.compare(moiKgm2, o.moiKgm2) == 0
                    && Double.compare(radiusM, o.radiusM) == 0
                    && Double.compare(gearRatio, o.gearRatio) == 0
                    && motorCount == o.motorCount
                    && java.util.Objects.equals(motor, o.motor)
                    && Double.compare(frictionTorqueNm, o.frictionTorqueNm) == 0
                    && Double.compare(kpPerRpm, o.kpPerRpm) == 0
                    && voltageCompensation == o.voltageCompensation;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(moiKgm2, radiusM, gearRatio, motorCount, motor, frictionTorqueNm, kpPerRpm, voltageCompensation);
        }

        @Override
        public String toString() {
            return "Params[moiKgm2=" + moiKgm2 + ", radiusM=" + radiusM + ", gearRatio=" + gearRatio + ", motorCount=" + motorCount + ", motor=" + motor + ", frictionTorqueNm=" + frictionTorqueNm + ", kpPerRpm=" + kpPerRpm + ", voltageCompensation=" + voltageCompensation + "]";
        }

        /** Highest flywheel speed reachable on a nominal battery with no load, rpm. */
        public double freeSpeedRpm() {
            return Physics.radPerSecToRpm(motor.freeSpeedRadPerSec() / gearRatio);
        }
    
    }

    private final Params p;
    private final DcMotor[] motors;
    private double omega;
    private double targetOmega;
    private double busVolts;
    private double windingCurrent;

    public Flywheel(Params p) {
        this.p = p;
        this.motors = new DcMotor[p.motorCount()];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new DcMotor(p.motor());
            motors[i].setBrake(false); // flywheels coast
        }
    }

    public Params params() { return p; }

    public void setTargetRpm(double rpm) { targetOmega = Physics.rpmToRadPerSec(Math.max(0, rpm)); }
    public double targetRpm() { return Physics.radPerSecToRpm(targetOmega); }
    public double rpm() { return Physics.radPerSecToRpm(omega); }
    public void setRpm(double rpm) { omega = Physics.rpmToRadPerSec(rpm); }
    public double busVolts() { return busVolts; }
    /** Total winding current of all launcher motors, A. */
    public double motorCurrent() { return windingCurrent * motors.length; }
    /** Current the launcher draws from the battery, A. */
    public double batteryCurrent() { return motors[0].duty() * windingCurrent * motors.length; }

    /** Duty that would hold the target speed in steady state at the given bus voltage. */
    double feedforward(double busV) {
        MotorSpec m = p.motor();
        double motorOmega = targetOmega * p.gearRatio();
        double torquePerMotor = p.frictionTorqueNm() / (p.motorCount() * p.gearRatio());
        double current = (torquePerMotor + m.frictionTorque()) / m.kt();
        double volts = m.kv() * motorOmega + current * m.resistance();
        double ref = p.voltageCompensation() ? Math.max(1, busV) : m.nominalVolts();
        return targetOmega <= 0 ? 0 : volts / ref;
    }

    private void command(double busV) {
        double duty = targetOmega <= 0 ? 0
                : feedforward(busV) + p.kpPerRpm() * (Physics.radPerSecToRpm(targetOmega - omega));
        duty = Math.max(0, Math.min(1, duty));
        for (DcMotor m : motors) m.setDuty(duty);
    }

    /** Battery current at bus voltage {@code v} (for solving the shared battery). */
    public double batteryCurrentAt(double v) {
        command(v);
        double motorOmega = omega * p.gearRatio();
        double total = 0;
        for (DcMotor m : motors) total += m.batteryCurrent(m.current(v, motorOmega));
        return total;
    }

    /** Steps with the flywheel as the only load besides {@code otherBatteryCurrentA}. */
    public void step(double dt, Battery battery, double otherBatteryCurrentA) {
        double v = battery.solve(volts -> otherBatteryCurrentA + batteryCurrentAt(volts));
        stepAt(dt, v);
        battery.drain(dt);
    }

    /** Steps at a known bus voltage (when something else solved the battery). */
    public void stepAt(double dt, double busV) {
        busVolts = busV;
        command(busV);
        double motorOmega = omega * p.gearRatio();
        double torque = 0;
        windingCurrent = 0;
        for (DcMotor m : motors) {
            double c = m.current(busV, motorOmega);
            windingCurrent = c;
            torque += m.torque(c, motorOmega) * p.gearRatio();
        }
        if (Math.abs(omega) > 1e-6) torque -= Math.copySign(p.frictionTorqueNm(), omega);
        else if (Math.abs(torque) <= p.frictionTorqueNm()) torque = 0;
        omega += torque / p.moiKgm2() * dt;
        if (omega < 0) omega = 0; // never commanded backwards; friction alone must not reverse it
    }

    /** Exit speed and wheel speed after firing one ball from wheel speed {@code omegaBefore}. */
    public static final class Shot {
        private final double exitSpeed;
        private final double omegaAfter;

        public Shot(double exitSpeed, double omegaAfter) {
            this.exitSpeed = exitSpeed;
            this.omegaAfter = omegaAfter;
        }

        public double exitSpeed() { return exitSpeed; }
        public double omegaAfter() { return omegaAfter; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Shot)) return false;
            Shot o = (Shot) other;
            return Double.compare(exitSpeed, o.exitSpeed) == 0
                    && Double.compare(omegaAfter, o.omegaAfter) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(exitSpeed, omegaAfter);
        }

        @Override
        public String toString() {
            return "Shot[exitSpeed=" + exitSpeed + ", omegaAfter=" + omegaAfter + "]";
        }
    }

    public static Shot shotFrom(double omegaBefore, double radiusM, double moiKgm2, double ballMassKg,
                                double exitSpeedRatio, double energyLossFactor) {
        double after = omegaBefore, v = 0;
        for (int i = 0; i < 4; i++) {
            double mid = 0.5 * (omegaBefore + after);
            v = exitSpeedRatio * mid * radiusM;
            double e = energyLossFactor * 0.5 * ballMassKg * v * v;
            after = Math.sqrt(Math.max(0, omegaBefore * omegaBefore - 2 * e / moiKgm2));
        }
        return new Shot(v, after);
    }

    /** Fires one ball now: returns its exit speed and slows the wheel. */
    public double fire(double ballMassKg, double exitSpeedRatio, double energyLossFactor) {
        Shot s = shotFrom(omega, p.radiusM(), p.moiKgm2(), ballMassKg, exitSpeedRatio, energyLossFactor);
        omega = s.omegaAfter();
        return s.exitSpeed();
    }
}
