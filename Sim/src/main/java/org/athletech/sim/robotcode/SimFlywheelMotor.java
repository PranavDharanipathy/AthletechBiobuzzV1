package org.athletech.sim.robotcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.Flywheel;

//what the robot's Flywheel class sees as its DcMotorEx: sim physics underneath, hub-style encoder readings on top
public final class SimFlywheelMotor implements DcMotorEx {

    private final Flywheel physics;
    private final double ticksPerMotorRev;
    private final double velocityWindow;

    private Direction direction = Direction.FORWARD;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private double power;
    private double motorRevs;

    private static final int RING = 8192;
    private final double[] ringTime = new double[RING], ringTicks = new double[RING];
    private int ringNewest = -1, ringCount = 0;
    private double now;

    /// velocityWindow = how far back the hub's velocity reading averages, seconds
    public SimFlywheelMotor(Flywheel physics, double ticksPerMotorRev, double velocityWindow) {

        this.physics = physics;
        this.ticksPerMotorRev = ticksPerMotorRev;
        this.velocityWindow = Math.max(velocityWindow, 1e-4);
        physics.setManualDuty(0);
    }

    public Flywheel physics() {
        return physics;
    }

    public void step(double t, double h, Battery battery, double otherCurrentA) {

        physics.step(h, battery, otherCurrentA);
        motorRevs += physics.rpm() / 60 * physics.params().gearRatio() * h;
        now = t + h;
        ringNewest = (ringNewest + 1) % RING;
        ringTime[ringNewest] = now;
        ringTicks[ringNewest] = motorRevs * ticksPerMotorRev;
        if (ringCount < RING) ringCount++;
    }

    private double sign() {
        return direction == Direction.FORWARD ? 1 : -1;
    }

    @Override
    public int getCurrentPosition() {
        return (int) Math.round(sign() * motorRevs * ticksPerMotorRev);
    }

    @Override
    public double getVelocity() {

        if (ringCount < 2) return 0;
        int newest = ringNewest, oldest = ringNewest;
        for (int k = 1; k < ringCount; k++) {
            int i = (ringNewest - k + RING) % RING;
            oldest = i;
            if (now - ringTime[i] >= velocityWindow) break;
        }
        double span = ringTime[newest] - ringTime[oldest];
        if (span <= 0) return 0;
        return sign() * Math.round((ringTicks[newest] - ringTicks[oldest]) / span);
    }

    @Override
    public void setPower(double power) {

        this.power = Math.max(-1, Math.min(1, power));
        physics.setManualDuty(sign() * this.power);
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {}
}
