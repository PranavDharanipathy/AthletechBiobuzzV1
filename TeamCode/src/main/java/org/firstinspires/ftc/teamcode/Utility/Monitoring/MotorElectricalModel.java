package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Electrical estimates for brushed DC motors. REV hubs cannot measure back-EMF directly, so it is
 * estimated two ways:
 *
 * <ol>
 *   <li><b>From velocity</b> (always available): back-EMF is proportional to speed, and a motor
 *   at its rated free speed generates roughly its rated voltage. The configured motor type gives
 *   free speed ({@code getMaxRPM}) and encoder resolution ({@code getTicksPerRev}), so
 *   {@code E = 12 V * velocity / freeSpeed}. Sign follows direction of rotation. Accuracy depends
 *   on the motor type in the robot configuration being correct.</li>
 *   <li><b>From current</b> (only if you set a winding resistance): {@code V = E + I*R}, so
 *   {@code E = power * V_battery - sign(power) * I * R}. Useful as a cross-check; it degrades at
 *   low power and while braking because the hub reports current magnitude only.</li>
 * </ol>
 *
 * The ratio {@code |E| / V_battery} shows how close a motor is to running out of voltage
 * headroom (near 1.0 = it cannot go faster on this battery).
 */
public final class MotorElectricalModel {

    /** FTC-legal motors are rated at 12 V. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    private MotorElectricalModel() {
    }

    /** Back-EMF in volts from encoder velocity (ticks/s), or NaN if it cannot be computed. */
    public static double backEmfFromVelocity(double ticksPerSecond, MotorConfigurationType type) {
        if (type == null || Double.isNaN(ticksPerSecond)) return Double.NaN;
        double ticksPerRev = type.getTicksPerRev();
        double freeRpm = type.getMaxRPM();
        if (!(ticksPerRev > 0) || !(freeRpm > 0)) return Double.NaN;
        double freeTicksPerSecond = ticksPerRev * freeRpm / 60.0;
        return NOMINAL_VOLTAGE * ticksPerSecond / freeTicksPerSecond;
    }

    /**
     * Back-EMF in volts from the electrical model, or NaN without a resistance, battery voltage,
     * current or a known power (duty cycle).
     */
    public static double backEmfFromCurrent(double power, double batteryVolts, double currentAmps,
                                            double windingOhms) {
        if (!(windingOhms > 0) || Double.isNaN(power) || Double.isNaN(batteryVolts)
                || Double.isNaN(currentAmps)) {
            return Double.NaN;
        }
        if (Math.abs(power) < 0.02) return Double.NaN; // coasting/braking: sign of current unknown
        double applied = power * batteryVolts;
        return applied - Math.signum(power) * currentAmps * windingOhms;
    }

    /** Electrical power drawn by the motor in watts: |duty| * V_battery * I. */
    public static double inputPowerWatts(double power, double batteryVolts, double currentAmps) {
        if (Double.isNaN(power) || Double.isNaN(batteryVolts) || Double.isNaN(currentAmps)) {
            return Double.NaN;
        }
        return Math.abs(power) * batteryVolts * currentAmps;
    }
}
