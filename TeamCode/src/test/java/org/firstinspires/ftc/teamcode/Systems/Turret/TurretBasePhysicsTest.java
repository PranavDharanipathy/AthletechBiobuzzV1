package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;

import static org.junit.Assert.*;

/** Physics-consistency checks from docs/PHYSICS_ERROR_BUDGET.md (findings C, I.1, I.2). */
public class TurretBasePhysicsTest {

    private static final class Rig {
        int position;
        double hubVelocity;
        double batteryVolts = 12;
        int batteryReads;
        final double[] powers = new double[2];
        final TurretBase controller;

        Rig() {
            DcMotorEx port = fake(DcMotorEx.class, (proxy, method, args) -> {
                if (method.getName().equals("getCurrentPosition")) return position;
                if (method.getName().equals("getVelocity")) return hubVelocity;
                throw new AssertionError(method);
            });
            VoltageSensor battery = fake(VoltageSensor.class, (proxy, method, args) -> {
                if (method.getName().equals("getVoltage")) { batteryReads++; return batteryVolts; }
                throw new AssertionError(method);
            });
            controller = new TurretBase(servo(0), servo(1), new Encoder(port), battery, 0.0);
            controller.setPositionalCoefficients(new TurretBasePIDFCoefficients(0.02, 0.02, new double[] {0, 0},
                    new double[] {0, 0}, 0, 0, 0, 10, new double[] {1, 1000}, 10,
                    new double[] {1, 1}, new double[] {0, 1000}, 1, 0, 0).withTuning(true));
        }

        CRServo servo(final int index) {
            return fake(CRServo.class, (proxy, method, args) -> {
                if (method.getName().equals("setPower")) { powers[index] = (Double) args[0]; return null; }
                if (method.getName().equals("setDirection")) return null;
                throw new AssertionError(method);
            });
        }
    }

    private static <T> T fake(Class<T> type, InvocationHandler handler) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, handler));
    }

    /** Servos run off a regulated rail: the command must not change with battery voltage. */
    @Test public void servoCommandDoesNotDependOnBatteryVoltage() {
        assertFalse(TurretConstants.TURRET_BATTERY_COMPENSATION);
        Rig full = new Rig();
        Rig sagging = new Rig();
        sagging.batteryVolts = 10.5;
        full.controller.setPosition(100);
        sagging.controller.setPosition(100);
        full.controller.update(1_000_000_000L);
        sagging.controller.update(1_000_000_000L);
        assertEquals(2.0 / 12, full.powers[0], 1e-12);
        assertEquals(full.powers[0], sagging.powers[0], 0);
        assertEquals("no blocking battery read in the loop", 0, full.batteryReads);
    }

    /** The hub reports velocity as a 16-bit short; above 32767 ticks/s (≈ 480 °/s) it wraps. */
    @Test public void hubVelocityWrapIsCorrected() {
        Rig rig = new Rig();
        long t0 = System.nanoTime();
        rig.position = 0;
        rig.controller.update(t0);
        double trueVelocity = 40000; // ticks/s ≈ 586 °/s at 68.27 ticks/°
        rig.position = (int) (trueVelocity * 0.020);
        rig.hubVelocity = trueVelocity - 65536; // what the short register reports
        rig.controller.update(t0 + 20_000_000L);
        assertEquals(trueVelocity, rig.controller.getVelocity(), 1e-9);
    }

    @Test public void overflowCorrectionLeavesInRangeValuesAlone() {
        assertEquals(1200, Encoder.correctVelocityOverflow(1200, 1180), 0);
        assertEquals(-30000, Encoder.correctVelocityOverflow(-30000, -29500), 0);
        assertEquals(-40000, Encoder.correctVelocityOverflow(25536, -39900), 0);
        assertEquals(1200, Encoder.correctVelocityOverflow(1200, Double.NaN), 0);
    }
}
