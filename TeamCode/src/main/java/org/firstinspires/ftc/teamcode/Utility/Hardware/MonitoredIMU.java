package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Drop-in {@link IMU} that counts and times every call and can cache reads.
 *
 * <pre>
 * IMU imu = new MonitoredIMU(hardwareMap, "imu", true);
 * </pre>
 *
 * IMU reads are I2C transactions (typically a few ms each) and are <b>not</b> covered by REV
 * hub bulk caching, so read caching here is often the single biggest loop-time win: heading
 * can be read from drive code, telemetry and a turn controller in the same loop for the cost of
 * one transaction.
 */
public class MonitoredIMU extends MonitoredDevice<IMU> implements IMU {

    private final CacheSlot yprSlot;
    private final CacheSlot orientationSlot;
    private final CacheSlot quaternionSlot;
    private final CacheSlot angularVelocitySlot;

    public MonitoredIMU(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, IMU.class, name), name, cacheReads);
    }

    public MonitoredIMU(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredIMU(IMU imu, String name, boolean cacheReads) {
        super(imu, name, "IMU", cacheReads);
        yprSlot = tracker.newSlot();
        orientationSlot = tracker.newSlot();
        quaternionSlot = tracker.newSlot();
        angularVelocitySlot = tracker.newSlot();
    }

    @Override
    public boolean initialize(Parameters parameters) {
        long t0 = tracker.start();
        boolean ok = device.initialize(parameters);
        tracker.write(HwOp.IMU_INITIALIZE, t0, false);
        return ok;
    }

    @Override
    public void resetYaw() {
        long t0 = tracker.start();
        device.resetYaw();
        tracker.write(HwOp.IMU_RESET_YAW, t0, false);
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        if (tracker.cacheHit(HwOp.GET_YAW_PITCH_ROLL, yprSlot, CacheSlot.NO_KEY)) {
            return (YawPitchRollAngles) yprSlot.o;
        }
        long t0 = tracker.start();
        YawPitchRollAngles v = device.getRobotYawPitchRollAngles();
        yprSlot.o = v;
        tracker.read(HwOp.GET_YAW_PITCH_ROLL, t0, yprSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        long key = ord(reference) * 10_000L + ord(order) * 100L + ord(angleUnit);
        if (tracker.cacheHit(HwOp.GET_ORIENTATION, orientationSlot, key)) {
            return (Orientation) orientationSlot.o;
        }
        long t0 = tracker.start();
        Orientation v = device.getRobotOrientation(reference, order, angleUnit);
        orientationSlot.o = v;
        tracker.read(HwOp.GET_ORIENTATION, t0, orientationSlot, key);
        return v;
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        if (tracker.cacheHit(HwOp.GET_QUATERNION, quaternionSlot, CacheSlot.NO_KEY)) {
            return (Quaternion) quaternionSlot.o;
        }
        long t0 = tracker.start();
        Quaternion v = device.getRobotOrientationAsQuaternion();
        quaternionSlot.o = v;
        tracker.read(HwOp.GET_QUATERNION, t0, quaternionSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        long key = ord(angleUnit);
        if (tracker.cacheHit(HwOp.GET_ANGULAR_VELOCITY, angularVelocitySlot, key)) {
            return (AngularVelocity) angularVelocitySlot.o;
        }
        long t0 = tracker.start();
        AngularVelocity v = device.getRobotAngularVelocity(angleUnit);
        angularVelocitySlot.o = v;
        tracker.read(HwOp.GET_ANGULAR_VELOCITY, t0, angularVelocitySlot, key);
        return v;
    }

    private static long ord(Enum<?> e) {
        return e == null ? 99 : e.ordinal();
    }
}
