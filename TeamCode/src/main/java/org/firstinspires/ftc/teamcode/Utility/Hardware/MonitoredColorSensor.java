package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drop-in color sensor. Implements {@link ColorSensor}, {@link NormalizedColorSensor} and
 * {@link DistanceSensor}, matching the REV Color Sensor V3, so it can stand in for whichever of
 * those you were using. Calling an interface the real sensor lacks throws
 * {@link UnsupportedOperationException}.
 *
 * <pre>
 * NormalizedColorSensor color = new MonitoredColorSensor(hardwareMap, "color", true);
 * </pre>
 */
public class MonitoredColorSensor extends MonitoredDevice<HardwareDevice>
        implements ColorSensor, NormalizedColorSensor, DistanceSensor {

    private final CacheSlot redSlot;
    private final CacheSlot greenSlot;
    private final CacheSlot blueSlot;
    private final CacheSlot alphaSlot;
    private final CacheSlot argbSlot;
    private final CacheSlot normalizedSlot;
    private final CacheSlot distanceSlot;
    private Boolean lastLed;
    private float lastGain = Float.NaN;

    public MonitoredColorSensor(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, HardwareDevice.class, name), name, cacheReads);
    }

    public MonitoredColorSensor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    /** @param sensor a {@link ColorSensor} and/or {@link NormalizedColorSensor} */
    public MonitoredColorSensor(HardwareDevice sensor, String name, boolean cacheReads) {
        super(sensor, name, "ColorSensor", cacheReads);
        if (!(sensor instanceof ColorSensor) && !(sensor instanceof NormalizedColorSensor)) {
            throw new IllegalArgumentException("'" + name + "' (" + sensor.getClass().getSimpleName()
                    + ") is not a color sensor");
        }
        redSlot = tracker.newSlot();
        greenSlot = tracker.newSlot();
        blueSlot = tracker.newSlot();
        alphaSlot = tracker.newSlot();
        argbSlot = tracker.newSlot();
        normalizedSlot = tracker.newSlot();
        distanceSlot = tracker.newSlot();
    }

    public boolean supportsRawColor() {
        return device instanceof ColorSensor;
    }

    public boolean supportsNormalizedColor() {
        return device instanceof NormalizedColorSensor;
    }

    public boolean supportsDistance() {
        return device instanceof DistanceSensor;
    }

    private ColorSensor raw() {
        if (device instanceof ColorSensor) return (ColorSensor) device;
        throw unsupported("ColorSensor");
    }

    private NormalizedColorSensor normalized() {
        if (device instanceof NormalizedColorSensor) return (NormalizedColorSensor) device;
        throw unsupported("NormalizedColorSensor");
    }

    private DistanceSensor distance() {
        if (device instanceof DistanceSensor) return (DistanceSensor) device;
        throw unsupported("DistanceSensor");
    }

    private UnsupportedOperationException unsupported(String what) {
        return new UnsupportedOperationException("'" + getName() + "' ("
                + device.getClass().getSimpleName() + ") is not a " + what);
    }

    // ------------------------------------------------------------------ ColorSensor

    @Override
    public int red() {
        return channel(HwOp.GET_RED, redSlot);
    }

    @Override
    public int green() {
        return channel(HwOp.GET_GREEN, greenSlot);
    }

    @Override
    public int blue() {
        return channel(HwOp.GET_BLUE, blueSlot);
    }

    @Override
    public int alpha() {
        return channel(HwOp.GET_ALPHA, alphaSlot);
    }

    @Override
    public int argb() {
        return channel(HwOp.GET_ARGB, argbSlot);
    }

    private int channel(HwOp op, CacheSlot slot) {
        ColorSensor s = raw();
        if (tracker.cacheHit(op, slot, CacheSlot.NO_KEY)) return slot.i;
        long t0 = tracker.start();
        int v;
        switch (op) {
            case GET_RED:
                v = s.red();
                break;
            case GET_GREEN:
                v = s.green();
                break;
            case GET_BLUE:
                v = s.blue();
                break;
            case GET_ALPHA:
                v = s.alpha();
                break;
            default:
                v = s.argb();
                break;
        }
        slot.i = v;
        tracker.read(op, t0, slot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public void enableLed(boolean enable) {
        ColorSensor s = raw();
        long t0 = tracker.start();
        s.enableLed(enable);
        tracker.write(HwOp.ENABLE_LED, t0, lastLed != null && lastLed == enable);
        lastLed = enable;
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        ColorSensor s = raw();
        long t0 = tracker.start();
        s.setI2cAddress(newAddress);
        tracker.write(HwOp.SET_I2C_ADDRESS, t0, false);
    }

    @Override
    public I2cAddr getI2cAddress() {
        ColorSensor s = raw();
        long t0 = tracker.start();
        I2cAddr v = s.getI2cAddress();
        tracker.read(HwOp.GET_I2C_ADDRESS, t0);
        return v;
    }

    // ------------------------------------------------------------------ NormalizedColorSensor

    @Override
    public NormalizedRGBA getNormalizedColors() {
        NormalizedColorSensor s = normalized();
        if (tracker.cacheHit(HwOp.GET_NORMALIZED_COLORS, normalizedSlot, CacheSlot.NO_KEY)) {
            return (NormalizedRGBA) normalizedSlot.o;
        }
        long t0 = tracker.start();
        NormalizedRGBA v = s.getNormalizedColors();
        normalizedSlot.o = v;
        tracker.read(HwOp.GET_NORMALIZED_COLORS, t0, normalizedSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public float getGain() {
        NormalizedColorSensor s = normalized();
        long t0 = tracker.start();
        float v = s.getGain();
        tracker.read(HwOp.GET_GAIN, t0);
        return v;
    }

    @Override
    public void setGain(float newGain) {
        NormalizedColorSensor s = normalized();
        long t0 = tracker.start();
        s.setGain(newGain);
        tracker.write(HwOp.SET_GAIN, t0, newGain == lastGain, normalizedSlot);
        lastGain = newGain;
    }

    // ------------------------------------------------------------------ DistanceSensor

    @Override
    public double getDistance(DistanceUnit unit) {
        DistanceSensor s = distance();
        if (tracker.cacheHit(HwOp.GET_DISTANCE, distanceSlot, CacheSlot.NO_KEY)) {
            return unit.fromMm(distanceSlot.d);
        }
        long t0 = tracker.start();
        double mm = s.getDistance(DistanceUnit.MM);
        distanceSlot.d = mm;
        tracker.read(HwOp.GET_DISTANCE, t0, distanceSlot, CacheSlot.NO_KEY);
        return unit.fromMm(mm);
    }
}
