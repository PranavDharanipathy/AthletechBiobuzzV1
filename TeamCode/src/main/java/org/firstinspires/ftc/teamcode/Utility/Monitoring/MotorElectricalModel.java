package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public final class MotorElectricalModel {

    public static final double NOMINAL_VOLTAGE = 12.0;

    private MotorElectricalModel() {
    }

    public static double backEmfFromVelocity(double ticksPerSecond, MotorConfigurationType type) {

        if (type == null || Double.isNaN(ticksPerSecond)) return Double.NaN;
        double ticksPerRev = type.getTicksPerRev();
        double freeRpm = type.getMaxRPM();
        if (!(ticksPerRev > 0) || !(freeRpm > 0)) return Double.NaN;
        double freeTicksPerSecond = ticksPerRev * freeRpm / 60.0;
        return NOMINAL_VOLTAGE * ticksPerSecond / freeTicksPerSecond;
    }

    public static double backEmfFromCurrent(double power, double batteryVolts, double currentAmps, double windingOhms) {

        if (!(windingOhms > 0) || Double.isNaN(power) || Double.isNaN(batteryVolts) || Double.isNaN(currentAmps)) {
            return Double.NaN;
        }
        if (Math.abs(power) < 0.02) return Double.NaN;
        double applied = power * batteryVolts;
        return applied - Math.signum(power) * currentAmps * windingOhms;
    }

    public static double inputPowerWatts(double power, double batteryVolts, double currentAmps) {

        if (Double.isNaN(power) || Double.isNaN(batteryVolts) || Double.isNaN(currentAmps)) {
            return Double.NaN;
        }
        return Math.abs(power) * batteryVolts * currentAmps;
    }
}
