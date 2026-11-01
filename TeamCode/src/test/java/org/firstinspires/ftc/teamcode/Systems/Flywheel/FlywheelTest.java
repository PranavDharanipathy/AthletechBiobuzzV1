package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.junit.Test;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

/** Exercises the production baseline controller with SDK-interface fakes, no robot. */
public class FlywheelTest {
    private static final class Rig {
        double motorPower;
        final Flywheel controller;

        Rig() throws Exception {
            DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("setPower")) { motorPower = (Double) args[0]; return null; }
                        if (method.getName().equals("getCurrentPosition")) return 0;
                        if (method.getName().equals("getPower")) return motorPower;
                        if (method.getName().equals("getVelocity")) return 0.0;
                        if (method.getReturnType() == void.class) return null;
                        throw new AssertionError("Unexpected motor call: " + method);
                    });
            VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(
                    VoltageSensor.class.getClassLoader(), new Class<?>[] {VoltageSensor.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("getVoltage")) return 12.0;
                        throw new AssertionError("Unexpected battery call: " + method);
                    });
            controller = new Flywheel(motor);
            set(controller, "batteryVoltageSensor", battery);
        }
    }

    private static FlywheelPIDVSCoefficients gains(double integral) {
        return new FlywheelPIDVSCoefficients(0.02, 0.01, integral, integral, 0, 0.01,
                0, 50, 10, 10, 1, -12, 12, 0, 0, -12, 12);
    }

    @Test public void disabledIntegralWithZeroBoundsDoesNotPoisonOutput() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocityPIDVSCoefficients(gains(0));
        rig.controller.setVelocity(100, true);
        rig.controller.update();
        assertEquals(0, rig.controller.i, 0);
        assertEquals(0.25, rig.motorPower, 1e-12); // (P=2 V + FF=1 V)/12 V
    }

    @Test public void gainsUseTheCurrentVelocitySample() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocityPIDVSCoefficients(gains(0.001));
        rig.controller.setVelocity(100, true);
        set(rig.controller, "currentVelocity", 100.0);
        rig.controller.update(); // first encoder sample establishes velocity 0
        assertEquals(0.02, rig.controller.kp, 0);
    }

    @Test public void resetStopsTheMotorImmediately() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocityPIDVSCoefficients(gains(0.001));
        rig.controller.setVelocity(100, true);
        rig.controller.update();
        assertTrue(rig.motorPower > 0);
        rig.controller.reset();
        assertEquals(0, rig.motorPower, 0);
        assertEquals(0, rig.controller.getPower(), 0);
    }

    @Test public void oppositeRotationIsNotReady() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(100, true);
        set(rig.controller, "firstTick", false);
        set(rig.controller, "currentVelocity", -100.0);
        set(rig.controller, "lastCurrentVelocity", -100.0);
        assertFalse(rig.controller.isAtVelocityAndStable(1, 1));
    }

    @Test public void stoppedOrUnobservedWheelIsNotReady() throws Exception {
        Rig rig = new Rig();
        assertFalse(rig.controller.isAtVelocityAndStable(1, 1));
        set(rig.controller, "firstTick", false);
        assertFalse(rig.controller.isAtVelocityAndStable(1, 1));
    }

    @Test public void signedReverseTargetCanBeReadyButManualModeCannot() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(-100, true);
        set(rig.controller, "firstTick", false);
        set(rig.controller, "currentVelocity", -100.0);
        set(rig.controller, "lastCurrentVelocity", -100.0);
        assertTrue(rig.controller.isAtVelocityAndStable(1, 1));
        assertFalse(rig.controller.isAtVelocityAndStable(Double.POSITIVE_INFINITY, 1));
        rig.controller.runMotor(Flywheel.RunningMotor.DISABLE);
        assertFalse(rig.controller.isAtVelocityAndStable(1, 1));
    }

    @Test public void negativeIntegralGainKeepsTheOutputBoundsOrdered() throws Exception {
        Rig rig = new Rig();
        FlywheelPIDVSCoefficients coefficients = gains(-0.1);
        coefficients.minI = -1;
        coefficients.maxI = 1;
        rig.controller.setVelocityPIDVSCoefficients(coefficients);
        rig.controller.setVelocity(100, true);
        Field field = Flywheel.class.getDeclaredField("errorSum");
        field.setAccessible(true);
        ((org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSum) field.get(rig.controller)).setSum(100);
        rig.controller.update();
        assertEquals(-1, rig.controller.i, 1e-12);
    }

    @Test public void manualPowerRejectsFaultsAndReportsTheCommand() throws Exception {
        Rig rig = new Rig();
        rig.controller.runMotor(Flywheel.RunningMotor.DISABLE);
        rig.controller.setPower(Double.NaN);
        assertEquals(0, rig.motorPower, 0);
        rig.controller.setPower(Double.POSITIVE_INFINITY);
        assertEquals(0, rig.motorPower, 0);
        rig.controller.setPower(5);
        assertEquals(1, rig.motorPower, 0);
        assertEquals(1, rig.controller.getPower(), 0);
    }

    private static void set(Object target, String name, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }
}
