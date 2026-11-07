package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//Sim stand-in for the real HubVoltage (that one walks the Lynx modules)
public final class HubVoltage {

    private HubVoltage() {}

    public static VoltageSensor forDevice(HardwareMap hardwareMap, HardwareDevice device) {
        return hardwareMap.simVoltageSensor;
    }
}
