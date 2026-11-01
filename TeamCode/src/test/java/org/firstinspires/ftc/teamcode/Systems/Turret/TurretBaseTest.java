package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.junit.Test;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

public class TurretBaseTest {
    private static final class Rig {
        final double[] powers = new double[2];
        final TurretBase controller;
        Rig() {
            DcMotorEx port = fake(DcMotorEx.class, (proxy, method, args) -> {
                if (method.getName().equals("getCurrentPosition")) return 0;
                throw new AssertionError(method);
            });
            VoltageSensor battery = fake(VoltageSensor.class, (proxy, method, args) -> {
                if (method.getName().equals("getVoltage")) return 12.0;
                throw new AssertionError(method);
            });
            controller = new TurretBase(servo(0), servo(1), new Encoder(port), battery, 0.0);
        }
        CRServo servo(final int index) {
            return fake(CRServo.class, (proxy, method, args) -> {
                if (method.getName().equals("setPower")) { powers[index] = (Double) args[0]; return null; }
                if (method.getName().equals("setDirection")) return null;
                throw new AssertionError(method);
            });
        }
    }
    private static <T> T fake(Class<T> type, java.lang.reflect.InvocationHandler handler) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, handler));
    }
    private static TurretBasePIDFCoefficients gains(double decay, double minI, double maxI) {
        return new TurretBasePIDFCoefficients(0.02, 0.02, new double[] {0, 0},
                new double[] {0, 0}, 0, 0, 0, 10, new double[] {1, 1000}, 10,
                new double[] {1, 1}, new double[] {0, 1000}, decay, minI, maxI).withTuning(true);
    }

    @Test public void zeroIntegralGainAndBoundsPreserveProportionalControl() {
        Rig rig = new Rig();
        rig.controller.setPositionalCoefficients(gains(1, 0, 0));
        rig.controller.setPosition(100);
        rig.controller.update();
        assertEquals(0, rig.controller.i, 0);
        assertEquals(2.0 / 12, rig.powers[0], 1e-12);
        assertEquals(rig.powers[0], rig.powers[1], 0);
    }

    @Test public void zeroDecayAtTargetDoesNotCreateNaN() {
        Rig rig = new Rig();
        rig.controller.setPositionalCoefficients(gains(0, -1, 1));
        rig.controller.update();
        assertEquals(0, rig.controller.f, 0);
        assertEquals(0, rig.powers[0], 0);
    }
}
