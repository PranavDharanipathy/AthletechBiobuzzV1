package org.athletech.sim.robot;

import java.util.Random;

public final class CrServoTurret {

    public static final class Params {

        public final double gain, timeConstant, deadTime, kineticFriction, breakawayFriction, velocityNoise, maxSpeed, maxAcceleration;
        public double startDelay;
        public double coastDecel = Double.POSITIVE_INFINITY, neutralDecel = Double.POSITIVE_INFINITY;

        public Params(double gain, double timeConstant, double deadTime, double kineticFriction, double breakawayFriction, double velocityNoise) {
            this(gain, timeConstant, deadTime, kineticFriction, breakawayFriction, velocityNoise, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        public Params(double gain, double timeConstant, double deadTime, double kineticFriction, double breakawayFriction, double velocityNoise, double maxSpeed, double maxAcceleration) {

            if (!(gain > 0) || !(timeConstant > 0) || deadTime < 0 || kineticFriction < 0 || breakawayFriction < 0) throw new IllegalArgumentException("turret gain and lag must be positive, dead time and friction not negative");
            this.gain = gain;
            this.timeConstant = timeConstant;
            this.deadTime = deadTime;
            this.kineticFriction = kineticFriction;
            this.breakawayFriction = Math.max(breakawayFriction, kineticFriction);
            this.velocityNoise = velocityNoise;
            this.maxSpeed = maxSpeed > 0 ? maxSpeed : Double.POSITIVE_INFINITY;
            this.maxAcceleration = maxAcceleration > 0 ? maxAcceleration : Double.POSITIVE_INFINITY;
        }
    }

    public final Params p;

    public double leftSign = 1, rightSign = 1, leftStrength = 1, rightStrength = 1, encoderSign = 1;
    public boolean jammed = false;

    private double x, w, robotAccel;
    private double leftNow, rightNow, pushingSince = Double.NaN;

    private static final int PIPE = 512;
    private final double[] pipeTime = new double[PIPE], pipeLeft = new double[PIPE], pipeRight = new double[PIPE];
    private int pipeHead = 0, pipeCount = 0;

    private final Random noise;

    private double syncAmplitude, syncHz, bandRms, now;
    private final double[] bandHz = {8, 11, 14}, bandPhase = new double[3];

    private final java.util.List<double[]> lanyards = new java.util.ArrayList<>();

    public void addLanyard(double tautTicks, double side, double voltsPerTick, double maxVolts, double dragVolts) {
        lanyards.add(new double[] {tautTicks, Math.signum(side), voltsPerTick, maxVolts, dragVolts});
    }

    public double lanyardVolts() {
        return lanyardVolts(x, Math.abs(w) >= 1 ? Math.signum(w) : 0);
    }

    public double lanyardVolts(double position, double direction) {

        double total = 0;
        for (double[] l : lanyards) {
            double stretch = l[1] * (position - l[0]);
            if (stretch <= 0) continue;
            total -= l[1] * Math.min(l[3], l[2] * stretch);
            total -= l[4] * direction;
        }
        return total;
    }

    public void place(double ticks) {

        x = ticks;
        w = 0;
    }

    public CrServoTurret(Params p, long seed) {

        this.p = p;
        noise = new Random(seed);
    }

    public void command(double now, double leftVolts, double rightVolts) {

        if (pipeCount == PIPE) {
            pipeHead = (pipeHead + 1) % PIPE;
            pipeCount--;
        }
        int k = (pipeHead + pipeCount) % PIPE;
        pipeTime[k] = now + p.deadTime;
        pipeLeft[k] = leftVolts;
        pipeRight[k] = rightVolts;
        pipeCount++;
    }

    public void setRobotAcceleration(double ticksPerSecondSquared) {
        robotAccel = ticksPerSecondSquared;
    }

    public void setVibration(double syncRmsTicks, double syncHz, double bandRmsTicks) {

        syncAmplitude = syncRmsTicks * Math.sqrt(2);
        this.syncHz = syncHz;
        bandRms = bandRmsTicks;
    }

    public double shake() {

        double band = 0;
        for (int k = 0; k < 3; k++) band += Math.sin(2 * Math.PI * bandHz[k] * now + bandPhase[k]);
        return syncAmplitude * Math.sin(2 * Math.PI * syncHz * now) + bandRms * band / Math.sqrt(1.5);
    }

    private double shakeRate() {

        double band = 0;
        for (int k = 0; k < 3; k++) band += 2 * Math.PI * bandHz[k] * Math.cos(2 * Math.PI * bandHz[k] * now + bandPhase[k]);
        return syncAmplitude * 2 * Math.PI * syncHz * Math.cos(2 * Math.PI * syncHz * now) + bandRms * band / Math.sqrt(1.5);
    }

    public void step(double now, double h) {

        this.now = now;
        if (bandRms > 0) for (int k = 0; k < 3; k++) bandPhase[k] += 3 * Math.sqrt(h) * noise.nextGaussian();

        while (pipeCount > 0 && pipeTime[pipeHead] <= now) {
            leftNow = pipeLeft[pipeHead];
            rightNow = pipeRight[pipeHead];
            pipeHead = (pipeHead + 1) % PIPE;
            pipeCount--;
        }
        if (jammed) {
            w = 0;
            return;
        }
        double u = (leftSign * leftStrength * leftNow + rightSign * rightStrength * rightNow) / 2 + lanyardVolts();
        if (Math.abs(w) < 1) {
            if (Math.abs(u) <= p.breakawayFriction) {
                w = 0;
                pushingSince = Double.NaN;
                return;
            }
            if (Double.isNaN(pushingSince)) pushingSince = now;
            if (now - pushingSince < p.startDelay - p.deadTime) {
                w = 0;
                return;
            }
            w = Math.signum(u) * 1.5;
        }
        pushingSince = Double.NaN;
        double next;
        if (Double.isInfinite(p.coastDecel)) {
            double wanted = Math.max(-p.maxSpeed, Math.min(p.maxSpeed, p.gain * (u - p.kineticFriction * Math.signum(w))));
            double rate = (wanted - w) / p.timeConstant - robotAccel;
            double cap = rate * w >= 0 ? p.maxAcceleration : 3 * p.maxAcceleration;
            next = w + h * Math.max(-cap, Math.min(cap, rate));
        }
        else {
            double wanted = Math.abs(u) > p.kineticFriction ? Math.signum(u) * Math.min(p.maxSpeed, p.gain * (Math.abs(u) - p.kineticFriction)) : 0;
            double rate;
            if (Math.abs(u) <= 0.1) rate = -Math.signum(w) * Math.min(p.neutralDecel, Math.abs(w) / h);
            else if (Math.signum(u) != Math.signum(w)) rate = Math.max(-3 * p.maxAcceleration, Math.min(3 * p.maxAcceleration, (wanted - w) / p.timeConstant));
            else if (Math.abs(wanted) >= Math.abs(w)) rate = Math.max(-p.maxAcceleration, Math.min(p.maxAcceleration, (wanted - w) / p.timeConstant));
            else rate = -Math.signum(w) * Math.min(p.coastDecel, Math.abs(w - wanted) / h);
            next = w + h * (rate - robotAccel);
        }
        if (Math.signum(next) != Math.signum(w) && (Math.abs(u) <= p.breakawayFriction || Math.signum(u) == Math.signum(w))) next = 0;
        w = next;
        x += w * h;
    }

    public double encoderPosition() {
        return encoderSign * Math.round(x + shake());
    }

    public double encoderVelocity() {
        return encoderSign * Math.round(w + shakeRate() + p.velocityNoise * noise.nextGaussian());
    }

    public double aim() {
        return x + shake();
    }

    public double position() {
        return x;
    }

    public double velocity() {
        return w;
    }

    public void reset() {

        x = w = robotAccel = leftNow = rightNow = 0;
        pushingSince = Double.NaN;
        pipeHead = pipeCount = 0;
    }
}
