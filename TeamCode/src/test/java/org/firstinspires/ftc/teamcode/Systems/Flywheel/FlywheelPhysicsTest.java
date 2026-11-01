package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

/** Physics-consistency checks from docs/PHYSICS_ERROR_BUDGET.md (findings B, C, D). */
public class FlywheelPhysicsTest {

    private static final class Rig {
        int position;
        double motorPower;
        final List<String> calls = new ArrayList<>();
        final Flywheel controller;

        Rig() throws Exception {
            DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class},
                    (proxy, method, args) -> {
                        switch (method.getName()) {
                            case "setPower": motorPower = (Double) args[0]; return null;
                            case "getCurrentPosition": calls.add("encoder"); return position;
                            case "getPower": return motorPower;
                            default:
                                if (method.getReturnType() == void.class) return null;
                                throw new AssertionError("Unexpected motor call: " + method);
                        }
                    });
            VoltageSensor battery = (VoltageSensor) Proxy.newProxyInstance(
                    VoltageSensor.class.getClassLoader(), new Class<?>[] {VoltageSensor.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("getVoltage")) { calls.add("battery"); return 12.0; }
                        throw new AssertionError("Unexpected battery call: " + method);
                    });
            controller = new Flywheel(motor);
            Field field = Flywheel.class.getDeclaredField("batteryVoltageSensor");
            field.setAccessible(true);
            field.set(controller, battery);
            controller.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
        }
    }

    private static final long T0 = 1_000_000_000L;
    private static long ms(double millis) { return T0 + Math.round(millis * 1e6); }

    /** The encoder sample must be timestamped before the blocking battery read, not after it. */
    @Test public void encoderIsReadBeforeTheBatteryRead() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(1000, true);
        rig.controller.update();
        assertEquals("encoder", rig.calls.get(0));
        assertTrue(rig.calls.indexOf("battery") > rig.calls.indexOf("encoder"));
    }

    /** With the bulk-read time supplied, velocity is exactly Δcounts / Δ(sample time). */
    @Test public void suppliedSampleTimeGivesExactVelocity() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(2000, true);
        rig.position = 0;
        rig.controller.update(ms(0));
        rig.position = 40;
        rig.controller.update(ms(20));
        assertEquals(2000, rig.controller.getCurrentVelocity(), 1e-9);
    }

    /** Production gains are in volts: the feedforward alone gives the physically required duty. */
    @Test public void productionFeedforwardIsInVolts() throws Exception {
        Rig rig = new Rig();
        double target = 0.6 * 2800; // 60 % of a 6000 rpm, 28-count motor's free speed
        double feedforwardVolts = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kv * target;
        double duty = feedforwardVolts / 12.0;
        assertEquals("duty ≈ speed / free speed", 0.6, duty, 0.6 * 0.15);
        // and the controller output reaches that order of magnitude on the first loop
        rig.controller.setVelocity(target, true);
        rig.controller.update(ms(0));
        assertTrue("power " + rig.motorPower, rig.motorPower > 0.5);
    }

    @Test public void windowVelocityAveragesOverTheWindow() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(1727, true);
        // 1727 ticks/s with 1-count quantization, 15 ms loops
        for (int k = 0; k <= 12; k++) {
            rig.position = (int) Math.floor(1727 * 0.015 * k);
            rig.controller.update(ms(15 * k));
        }
        double window = rig.controller.getWindowVelocity(0.10);
        assertEquals(1727, window, 1727 * 0.01);
        assertTrue(rig.controller.isReady(0.01));
        rig.controller.setVelocity(1727 * 1.02, false);
        assertFalse("2 % slow is not ready", rig.controller.isReady(0.01));
    }

    @Test public void windowNeedsAFullSpanAndClearsOnReset() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(1000, true);
        rig.controller.update(ms(0));
        rig.position = 50;
        rig.controller.update(ms(50));
        assertTrue(Double.isNaN(rig.controller.getWindowVelocity(0.10)));
        assertFalse(rig.controller.isReady(0.01));
        rig.position = 100;
        rig.controller.update(ms(100));
        assertEquals(1000, rig.controller.getWindowVelocity(0.10), 1e-9);
        rig.controller.reset();
        assertTrue(Double.isNaN(rig.controller.getWindowVelocity(0.10)));
    }

    @Test public void reverseRotationIsNeverReady() throws Exception {
        Rig rig = new Rig();
        rig.controller.setVelocity(1000, true);
        for (int k = 0; k <= 10; k++) {
            rig.position = -10 * k;
            rig.controller.update(ms(10 * k));
        }
        assertFalse(rig.controller.isReady(0.5));
    }
}
