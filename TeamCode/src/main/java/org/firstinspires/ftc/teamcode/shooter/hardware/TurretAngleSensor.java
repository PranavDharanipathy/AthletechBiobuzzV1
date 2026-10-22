package org.firstinspires.ftc.teamcode.shooter.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.shooter.core.MathUtil;

/**
 * Turret angle source (design D-6). The encoder hardware is not decided yet, so the controller only sees
 * this interface. Angle: turret radians, CCW positive, relative to robot forward, continuous (no wrap).
 * NaN = no valid reading.
 */
public interface TurretAngleSensor {

    /** Reads the current turret angle (call once per loop, after the bulk-cache clear). */
    double readAngle();

    /** Declares the current physical angle (homing). */
    void setCurrentAngle(double angleRad);

    /** Creates the sensor selected in {@link ShooterConfig}, homed to TURRET_START_ANGLE_DEG. */
    static TurretAngleSensor fromConfig(com.qualcomm.robotcore.hardware.HardwareMap hw) {
        double start = Math.toRadians(ShooterConfig.TURRET_START_ANGLE_DEG);
        if (ShooterConfig.TURRET_SENSOR == ShooterConfig.TurretSensorType.ANALOG) {
            return new Analog(hw.get(AnalogInput.class, ShooterConfig.TURRET_ANALOG), ShooterConfig.TURRET_ANALOG_MIN_V,
                    ShooterConfig.TURRET_ANALOG_MAX_V, ShooterConfig.TURRET_ANALOG_RATIO, start);
        }
        return new Quadrature(hw.get(DcMotorEx.class, ShooterConfig.TURRET_ENCODER_PORT),
                ShooterConfig.TURRET_COUNTS_PER_REV, ShooterConfig.TURRET_ENCODER_SIGN, start);
    }

    /** Quadrature encoder wired into a motor port's encoder input (e.g. REV Through Bore). */
    final class Quadrature implements TurretAngleSensor {
        private final DcMotorEx port;
        private final double radPerCount;
        private final int sign;
        private double offset;

        public Quadrature(DcMotorEx port, double countsPerTurretRev, int sign, double startAngleRad) {
            this.port = port;
            this.radPerCount = MathUtil.TAU / countsPerTurretRev;
            this.sign = sign >= 0 ? 1 : -1;
            setCurrentAngle(startAngleRad);
        }

        @Override
        public double readAngle() {
            return sign * port.getCurrentPosition() * radPerCount + offset;
        }

        @Override
        public void setCurrentAngle(double angleRad) {
            offset = angleRad - sign * port.getCurrentPosition() * radPerCount;
        }
    }

    /**
     * Analog absolute sensor (e.g. an analog encoder, or an Axon servo's analog feedback). If the sensor turns
     * {@code ratio} times per turret revolution, each reading is ambiguous across {@code ratio} sectors. The
     * sector is fixed by the declared start angle, and the reading is then unwrapped continuously.
     */
    final class Analog implements TurretAngleSensor {
        private final AnalogInput input;
        private final double vMin, vMax, ratio;
        private double lastSensor = Double.NaN;  // rad within one sensor turn
        private double sensorTurns;              // accumulated (continuous) sensor angle
        private double offset;

        public Analog(AnalogInput input, double vMin, double vMax, double ratio, double startAngleRad) {
            this.input = input;
            this.vMin = vMin;
            this.vMax = vMax;
            this.ratio = ratio;
            setCurrentAngle(startAngleRad);
        }

        private double sensorAngle() {
            double v = input.getVoltage();
            if (!MathUtil.isFinite(v) || v < vMin - 0.1 || v > vMax + 0.1) return Double.NaN; // disconnected
            return (v - vMin) / (vMax - vMin) * MathUtil.TAU;
        }

        @Override
        public double readAngle() {
            double s = sensorAngle();
            if (Double.isNaN(s)) return Double.NaN;
            if (Double.isNaN(lastSensor)) {
                lastSensor = s;
                sensorTurns = s;
            } else {
                sensorTurns += MathUtil.wrapPi(s - lastSensor);
                lastSensor = s;
            }
            return sensorTurns / ratio + offset;
        }

        @Override
        public void setCurrentAngle(double angleRad) {
            double s = sensorAngle();
            if (!Double.isNaN(s)) {
                lastSensor = s;
                sensorTurns = s;
            }
            offset = angleRad - sensorTurns / ratio;
        }
    }
}
