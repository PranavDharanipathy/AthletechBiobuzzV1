package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalDistanceSensor {

    private final DigitalChannel sensor;

    public DigitalDistanceSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(DigitalChannel.class, name);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean withinDistance() {
        return !sensor.getState();
    }

    public boolean beyondDistance() {
        return sensor.getState();
    }
}
