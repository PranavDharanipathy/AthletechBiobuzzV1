package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MonitoredVoltageSensor extends MonitoredDevice<VoltageSensor> implements VoltageSensor {

    private final CacheSlot voltageSlot;

    public MonitoredVoltageSensor(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, VoltageSensor.class, name), name, cacheReads);
    }

    public MonitoredVoltageSensor(VoltageSensor sensor, String name, boolean cacheReads) {

        super(sensor, name, "VoltageSensor", cacheReads);
        voltageSlot = tracker.newSlot();
    }

    @Override
    public double getVoltage() {

        if (tracker.cacheHit(HwOp.GET_VOLTAGE, voltageSlot, CacheSlot.NO_KEY)) return voltageSlot.d;
        long t0 = tracker.start();
        double v = device.getVoltage();
        voltageSlot.d = v;
        tracker.v(v);
        tracker.read(HwOp.GET_VOLTAGE, t0, voltageSlot, CacheSlot.NO_KEY);
        return v;
    }
}
