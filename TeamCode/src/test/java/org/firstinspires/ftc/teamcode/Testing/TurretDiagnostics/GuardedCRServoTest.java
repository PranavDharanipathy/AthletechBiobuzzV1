package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBasePIDFCoefficients;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class GuardedCRServoTest {

    /** Fake servo recording every power written and the last direction set. */
    private static final class FakeServo {
        final List<Double> writes = new ArrayList<>();
        DcMotorSimple.Direction direction;
        final CRServo servo = fake(CRServo.class, (proxy, method, args) -> {
            switch (method.getName()) {
                case "setPower": writes.add((Double) args[0]); return null;
                case "setDirection": direction = (DcMotorSimple.Direction) args[0]; return null;
                case "getDirection": return direction;
                case "getDeviceName": return "fake";
                default: throw new AssertionError(method);
            }
        });
    }

    private static <T> T fake(Class<T> type, InvocationHandler handler) {
        return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, handler));
    }

    @Test public void writesNeutralUntilAllowed() {
        FakeServo f = new FakeServo();
        GuardedCRServo g = new GuardedCRServo(f.servo);
        assertFalse(g.everWritten());
        g.setPower(0.8);
        assertEquals(0.8, g.requested(), 0);
        assertEquals(0, g.applied(), 0);
        assertEquals(0, f.writes.get(0), 0);
    }

    @Test public void clampsNonFiniteAndStops() {
        FakeServo f = new FakeServo();
        GuardedCRServo g = new GuardedCRServo(f.servo);
        g.allow(0.15);
        g.setPower(0.5);
        assertEquals(0.15, g.applied(), 0);
        assertTrue(g.saturated());
        g.setPower(-0.5);
        assertEquals(-0.15, g.applied(), 0);
        g.setPower(0.1);
        assertFalse(g.saturated());
        assertEquals(0.1, g.applied(), 0);
        g.setPower(Double.NaN);
        assertEquals(0, g.applied(), 0);
        g.setPower(Double.POSITIVE_INFINITY);
        assertEquals(0, g.applied(), 0);
        g.setPower(0.1);
        g.stop();
        assertFalse(g.isOpen());
        assertEquals(0, f.writes.get(f.writes.size() - 1), 0);
        g.setPower(0.1);
        assertEquals("stays stopped until allowed again", 0, g.applied(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void refusesInvalidLimit() {
        new GuardedCRServo(new FakeServo().servo).allow(Double.NaN);
    }

    @Test public void directionPassesThroughUnchanged() {
        FakeServo f = new FakeServo();
        GuardedCRServo g = new GuardedCRServo(f.servo);
        g.setDirection(DcMotorSimple.Direction.REVERSE);
        assertEquals(DcMotorSimple.Direction.REVERSE, f.direction);
        assertEquals(DcMotorSimple.Direction.REVERSE, g.getDirection());
    }

    /** The unmodified baseline controller can never exceed the guard's limit, on either servo. */
    @Test public void baselineControllerOutputIsClampedByTheGuard() {
        FakeServo left = new FakeServo();
        FakeServo right = new FakeServo();
        GuardedCRServo gl = new GuardedCRServo(left.servo);
        GuardedCRServo gr = new GuardedCRServo(right.servo);
        DcMotorEx port = fake(DcMotorEx.class, (proxy, method, args) -> {
            if (method.getName().equals("getCurrentPosition")) return 0;
            throw new AssertionError(method);
        });
        VoltageSensor battery = fake(VoltageSensor.class, (proxy, method, args) -> {
            if (method.getName().equals("getVoltage")) return 12.0;
            throw new AssertionError(method);
        });
        TurretBase turret = new TurretBase(gl, gr, new Encoder(port), battery, 0.0);
        turret.setPositionalCoefficients(new TurretBasePIDFCoefficients(1, 1, new double[] {0, 0},
                new double[] {0, 0}, 0, 0, 0, 10, new double[] {1, 1000}, 10,
                new double[] {1, 1}, new double[] {0, 1000}, 1, -1, 1).withTuning(true));
        gl.allow(0.1);
        gr.allow(0.1);
        turret.setPosition(1000);
        turret.update();
        // TurretBase saturates its own output at 1; the raw p+i+d+f terms are recorded separately.
        assertEquals("controller asked for full power", 1.0, turret.getPower(), 0);
        assertTrue(turret.p > 1);
        assertEquals(0.1, gl.applied(), 0);
        assertEquals(0.1, gr.applied(), 0);
        assertTrue(gl.saturated());
        gl.stop();
        gr.stop();
        turret.update();
        assertEquals(0, left.writes.get(left.writes.size() - 1), 0);
        assertEquals(0, right.writes.get(right.writes.size() - 1), 0);
    }
}
