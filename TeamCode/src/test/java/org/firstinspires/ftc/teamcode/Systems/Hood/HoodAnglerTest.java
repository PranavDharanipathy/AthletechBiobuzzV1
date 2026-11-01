package org.firstinspires.ftc.teamcode.Systems.Hood;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.HoodConstants;
import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Proxy;

import static org.junit.Assert.*;

public class HoodAnglerTest {

    private static final class FakeServo {
        double position;
        Servo.Direction direction;
        final Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[] {Servo.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition": position = (Double) args[0]; return null;
                        case "getPosition": return position;
                        case "setDirection": direction = (Servo.Direction) args[0]; return null;
                        case "getDirection": return direction;
                        default: throw new AssertionError(method);
                    }
                });
    }

    @After public void clearTrims() {
        HoodConstants.LEFT_HOOD_TRIM = 0;
        HoodConstants.RIGHT_HOOD_TRIM = 0;
    }

    @Test public void servosAreMirroredAndTrimmedIndependently() {
        FakeServo left = new FakeServo(), right = new FakeServo();
        HoodAngler hood = new HoodAngler(left.servo, right.servo);
        assertNotEquals(left.direction, right.direction);

        hood.setPosition(0.4);
        assertEquals(0.4, left.position, 0);
        assertEquals(0.4, right.position, 0);

        HoodConstants.RIGHT_HOOD_TRIM = 0.012;
        hood.setPosition(0.4);
        assertEquals(0.4, left.position, 0);
        assertEquals(0.412, right.position, 1e-12);
        assertEquals(0.4, hood.getPosition(), 1e-12);
    }

    @Test public void trimmedCommandsStayInServoRange() {
        FakeServo left = new FakeServo(), right = new FakeServo();
        HoodAngler hood = new HoodAngler(left.servo, right.servo);
        HoodConstants.LEFT_HOOD_TRIM = -0.05;
        HoodConstants.RIGHT_HOOD_TRIM = 0.05;
        hood.setPosition(1.0);
        assertEquals(0.95, left.position, 1e-12);
        assertEquals(1.0, right.position, 0);
        hood.setPosition(0.0);
        assertEquals(0.0, left.position, 0);
        assertEquals(0.05, right.position, 1e-12);
    }
}
