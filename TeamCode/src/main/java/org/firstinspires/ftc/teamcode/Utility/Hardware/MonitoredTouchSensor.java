package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Drop-in {@link TouchSensor} that counts and times every read and can cache them.
 *
 * <pre>
 * TouchSensor limit = new MonitoredTouchSensor(hardwareMap, "liftLimit", true);
 * </pre>
 */
public class MonitoredTouchSensor extends MonitoredDevice<TouchSensor> implements TouchSensor {

    private final CacheSlot pressedSlot;
    private final CacheSlot valueSlot;

    public MonitoredTouchSensor(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, TouchSensor.class, name), name, cacheReads);
    }

    public MonitoredTouchSensor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredTouchSensor(TouchSensor sensor, String name, boolean cacheReads) {
        super(sensor, name, "TouchSensor", cacheReads);
        pressedSlot = tracker.newSlot();
        valueSlot = tracker.newSlot();
    }

    @Override
    public boolean isPressed() {
        if (tracker.cacheHit(HwOp.IS_PRESSED, pressedSlot, CacheSlot.NO_KEY)) return pressedSlot.b;
        long t0 = tracker.start();
        boolean v = device.isPressed();
        pressedSlot.b = v;
        tracker.read(HwOp.IS_PRESSED, t0, pressedSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public double getValue() {
        if (tracker.cacheHit(HwOp.GET_TOUCH_VALUE, valueSlot, CacheSlot.NO_KEY)) return valueSlot.d;
        long t0 = tracker.start();
        double v = device.getValue();
        valueSlot.d = v;
        tracker.read(HwOp.GET_TOUCH_VALUE, t0, valueSlot, CacheSlot.NO_KEY);
        return v;
    }
}
