package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MonitoredDistanceSensor extends MonitoredDevice<DistanceSensor> implements DistanceSensor {

    private final CacheSlot distanceSlot;

    public MonitoredDistanceSensor(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, DistanceSensor.class, name), name, cacheReads);
    }

    public MonitoredDistanceSensor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredDistanceSensor(DistanceSensor sensor, String name, boolean cacheReads) {

        super(sensor, name, "DistanceSensor", cacheReads);
        distanceSlot = tracker.newSlot();
    }

    @Override
    public double getDistance(DistanceUnit unit) {

        if (tracker.cacheHit(HwOp.GET_DISTANCE, distanceSlot, CacheSlot.NO_KEY)) {
            return unit.fromMm(distanceSlot.d);
        }
        long t0 = tracker.start();
        double mm = device.getDistance(DistanceUnit.MM);
        distanceSlot.d = mm;
        tracker.read(HwOp.GET_DISTANCE, t0, distanceSlot, CacheSlot.NO_KEY);
        return unit.fromMm(mm);
    }
}
