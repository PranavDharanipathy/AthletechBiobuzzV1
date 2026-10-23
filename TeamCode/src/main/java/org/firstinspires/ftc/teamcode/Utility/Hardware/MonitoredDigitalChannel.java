package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Drop-in {@link DigitalChannel} (limit switches, beam breaks, LEDs, ...) that counts and times
 * every call and can cache reads.
 *
 * <pre>
 * DigitalChannel beam = new MonitoredDigitalChannel(hardwareMap, "beamBreak", true);
 * </pre>
 */
public class MonitoredDigitalChannel extends MonitoredDevice<DigitalChannel> implements DigitalChannel {

    private final CacheSlot stateSlot;
    private Boolean lastState;
    private Mode lastMode;

    public MonitoredDigitalChannel(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, DigitalChannel.class, name), name, cacheReads);
    }

    public MonitoredDigitalChannel(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredDigitalChannel(DigitalChannel channel, String name, boolean cacheReads) {
        super(channel, name, "DigitalChannel", cacheReads);
        stateSlot = tracker.newSlot();
    }

    @Override
    public Mode getMode() {
        long t0 = tracker.start();
        Mode v = device.getMode();
        tracker.read(HwOp.GET_MODE, t0);
        return v;
    }

    @Override
    public void setMode(Mode mode) {
        long t0 = tracker.start();
        device.setMode(mode);
        tracker.write(HwOp.SET_MODE, t0, mode == lastMode);
        lastMode = mode;
    }

    @Deprecated
    @Override
    @SuppressWarnings("deprecation")
    public void setMode(DigitalChannelController.Mode mode) {
        long t0 = tracker.start();
        device.setMode(mode);
        tracker.write(HwOp.SET_MODE, t0, false);
        lastMode = null;
    }

    @Override
    public boolean getState() {
        if (tracker.cacheHit(HwOp.GET_STATE, stateSlot, CacheSlot.NO_KEY)) return stateSlot.b;
        long t0 = tracker.start();
        boolean v = device.getState();
        stateSlot.b = v;
        tracker.read(HwOp.GET_STATE, t0, stateSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public void setState(boolean state) {
        long t0 = tracker.start();
        device.setState(state);
        tracker.write(HwOp.SET_STATE, t0, lastState != null && lastState == state, stateSlot);
        lastState = state;
    }
}
