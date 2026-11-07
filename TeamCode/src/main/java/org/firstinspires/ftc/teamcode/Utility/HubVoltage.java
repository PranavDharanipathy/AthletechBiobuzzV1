package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public final class HubVoltage {

    private HubVoltage() {}

    public static VoltageSensor forDevice(HardwareMap hardwareMap, HardwareDevice device) {

        final String deviceInfo = device.getConnectionInfo();
        
        VoltageSensor first = null;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {

            if (first == null) first = sensor;
            String hubInfo = sensor.getConnectionInfo();
            if (deviceInfo != null && hubInfo != null && deviceInfo.startsWith(hubInfo + ";")) return sensor;
        }

        return first;
    }
}
