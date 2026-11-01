package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

/** Real wrapper/cache code with fake SDK interfaces; no background session or robot I/O. */
public class ReadCacheTest {
    private boolean enabled;
    private long maxAge;
    @Before public void configure() {
        enabled = HardwareMonitor.CONFIG.enabled;
        maxAge = HardwareMonitor.CONFIG.cacheMaxAgeMs;
        HardwareMonitor.CONFIG.enabled = false;
        HardwareMonitor.CONFIG.cacheMaxAgeMs = 1000000;
    }
    @After public void restore() throws Exception {
        HardwareMonitor.CONFIG.enabled = enabled;
        HardwareMonitor.CONFIG.cacheMaxAgeMs = maxAge;
        // Wrappers register even when recording is disabled; clean up the fake devices.
        HardwareMonitor monitor = HardwareMonitor.get();
        java.lang.reflect.Method reset = HardwareMonitor.class.getDeclaredMethod("resetPendingLocked");
        reset.setAccessible(true);
        synchronized (monitor) { reset.invoke(monitor); }
    }

    private static final class Motor {
        int position = 42, positionReads, currentReads;
        double power;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getConnectionInfo": return "fake motor port";
                        case "getCurrentPosition": positionReads++; return position;
                        case "getCurrent": currentReads++; assertEquals(CurrentUnit.AMPS, args[0]); return 2.5;
                        case "setPower": power = (Double) args[0]; return null;
                        case "getPower": return power;
                        case "setMode": if (args[0] == DcMotor.RunMode.STOP_AND_RESET_ENCODER) position = 0; return null;
                        default: throw new AssertionError(method);
                    }
                });
    }

    @Test public void readsShareOneSampleUntilTheNextLoop() {
        Motor motor = new Motor();
        MonitoredDcMotorEx wrapper = new MonitoredDcMotorEx(motor.device, "cache-loop", true);
        assertEquals(42, wrapper.getCurrentPosition());
        motor.position = 99;
        assertEquals(42, wrapper.getCurrentPosition());
        assertEquals(1, motor.positionReads);
        HardwareMonitor.get().update(); // disabled monitoring still advances the cache cycle
        assertEquals(99, wrapper.getCurrentPosition());
        assertEquals(2, motor.positionReads);
    }

    @Test public void resetAndPowerWritesInvalidateTheAffectedReads() {
        Motor motor = new Motor();
        MonitoredDcMotorEx wrapper = new MonitoredDcMotorEx(motor.device, "cache-write", true);
        wrapper.getCurrentPosition();
        wrapper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        assertEquals(0, wrapper.getCurrentPosition());
        assertEquals(2, motor.positionReads);
        assertEquals(0, wrapper.getPower(), 0);
        wrapper.setPower(0.4);
        assertEquals(0.4, wrapper.getPower(), 0);
    }

    @Test public void currentUnitConversionUsesOneHardwareTransaction() {
        Motor motor = new Motor();
        MonitoredDcMotorEx wrapper = new MonitoredDcMotorEx(motor.device, "cache-units", true);
        assertEquals(2.5, wrapper.getCurrent(CurrentUnit.AMPS), 0);
        assertEquals(2500, wrapper.getCurrent(CurrentUnit.MILLIAMPS), 0);
        assertEquals(1, motor.currentReads);
    }

    @Test public void maxAgeAndRuntimeDisableCannotReturnStaleReads() {
        Motor motor = new Motor();
        MonitoredDcMotorEx wrapper = new MonitoredDcMotorEx(motor.device, "cache-age", true);
        wrapper.getCurrentPosition();
        HardwareMonitor.CONFIG.cacheMaxAgeMs = 1;
        wrapper.positionSlot.timeNs = System.nanoTime() - 1000000000L;
        motor.position = 7;
        assertEquals(7, wrapper.getCurrentPosition());
        wrapper.setCachingReads(false);
        motor.position = 8;
        assertEquals(8, wrapper.getCurrentPosition());
        wrapper.setCachingReads(true);
        motor.position = 9;
        assertEquals(9, wrapper.getCurrentPosition());
    }

    @Test public void yawResetInvalidatesTheSameLoopImuSample() {
        final int[] reads = {0};
        final double[] yaw = {30};
        IMU imu = (IMU) Proxy.newProxyInstance(IMU.class.getClassLoader(), new Class<?>[] {IMU.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("getConnectionInfo")) return "fake IMU";
                    if (method.getName().equals("resetYaw")) { yaw[0] = 0; return null; }
                    if (method.getName().equals("getRobotYawPitchRollAngles")) {
                        reads[0]++;
                        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw[0], 0, 0, System.nanoTime());
                    }
                    throw new AssertionError(method);
                });
        MonitoredIMU wrapper = new MonitoredIMU(imu, "cache-imu", true);
        assertEquals(30, wrapper.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0);
        wrapper.getRobotYawPitchRollAngles();
        assertEquals(1, reads[0]);
        wrapper.resetYaw();
        assertEquals(0, wrapper.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 0);
        assertEquals(2, reads[0]);
    }
}
