package org.athletech.sim.scenarios;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.athletech.sim.robotcode.SimClock;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FeedGate;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

//a match's worth of bursts against the real Flywheel (and FeedGate), on the wheel the 2028-05-15 logs show: it speeds up at
//accel per volt over what holds it but can't be braked (only coasts down), the hub reports a 50 ms count read a bit late, the
//command lands mid-loop, loops jitter, the battery drops over the match and sags when the drive pulls, balls take speed
final class FlywheelBursts {

    static final double H = 0.001, WINDOW = 0.05;

    static final class Config {

        boolean observer = true, gate;
        long seed = 1;
        double seconds = 300, burstsEvery = 6, balls = 3;
        //the real wheel, which the code's model may not match
        double accel = 165, kv = 0.00475, ks = 0.42, coast = 0.7, readAgeMax = 0.012, rInternal = 0.035, rMotor = 0.3;
        double startVolts = 13.3, endVolts = 11.0, startLoad = -0.2, endLoad = -0.7, driveAmps = 25, drivePerSecond = 0.5;
        double ballDrop = 95, ballDropSpread = 15, ballSeconds = 0.025;
        double firstContact = 0.24, interval = 0.30, intervalSpread = 0.03, commit = 0.10;
        double loopMedian = 0.034, loopSpread = 0.35;
        //the CELL's room under / over the aim-point speed at exit gain 0.6 (docs/FIELD.md: 0.3-1 all fit the 2028-05-15 video)
        double below = 39, above = 28, windowSpread = 0.15;
        //the table's speed - the speed that hits the aim point; each ball's own exit difference, as wheel speed
        double tableError, ballSpread;
        //how long before the trigger the robot got to its spot (the target jumps then)
        double settle = 1.5;
        boolean trace;

        //the wheel, balls, battery and loops the robot logs show (tools/flywheel_fit.py); the defaults above where there's no fit
        static Config fitted() {

            Config c = new Config();
            java.io.File file = new java.io.File("../tuning-data/flywheel-fit/plant.properties");
            if (!file.isFile()) file = new java.io.File("tuning-data/flywheel-fit/plant.properties");
            if (!file.isFile()) return c;
            java.util.Properties p = new java.util.Properties();
            try (java.io.FileInputStream in = new java.io.FileInputStream(file)) {
                p.load(in);
            }
            catch (java.io.IOException e) {
                return c;
            }
            boolean wheel = "1".equals(p.getProperty("wheelFit", "0").trim());
            for (String key : p.stringPropertyNames()) {
                if (key.equals("wheelFit")) continue;
                if (!wheel && (key.equals("accel") || key.equals("coast") || key.equals("kv") || key.equals("ks") || key.equals("readAgeMax"))) continue;
                try {
                    Config.class.getDeclaredField(key).setDouble(c, Double.parseDouble(p.getProperty(key).trim()));
                }
                catch (ReflectiveOperationException | NumberFormatException e) {
                    throw new IllegalStateException("plant.properties: " + key, e);
                }
            }
            return c;
        }

        Config copy() {

            Config c = new Config();
            c.observer = observer;
            c.gate = gate;
            c.seed = seed;
            c.seconds = seconds;
            c.burstsEvery = burstsEvery;
            c.balls = balls;
            c.accel = accel;
            c.kv = kv;
            c.ks = ks;
            c.coast = coast;
            c.readAgeMax = readAgeMax;
            c.rInternal = rInternal;
            c.rMotor = rMotor;
            c.startVolts = startVolts;
            c.endVolts = endVolts;
            c.startLoad = startLoad;
            c.endLoad = endLoad;
            c.driveAmps = driveAmps;
            c.drivePerSecond = drivePerSecond;
            c.ballDrop = ballDrop;
            c.ballDropSpread = ballDropSpread;
            c.ballSeconds = ballSeconds;
            c.firstContact = firstContact;
            c.interval = interval;
            c.intervalSpread = intervalSpread;
            c.commit = commit;
            c.loopMedian = loopMedian;
            c.loopSpread = loopSpread;
            c.below = below;
            c.above = above;
            c.windowSpread = windowSpread;
            c.tableError = tableError;
            c.ballSpread = ballSpread;
            c.settle = settle;
            c.trace = trace;
            return c;
        }
    }

    static final class Ball {

        int burst, index;
        double time, error, below, above, volts, shift;

        boolean miss() {
            return error + shift < -below || error + shift > above;
        }
    }

    static final class Result {

        final List<Ball> balls = new ArrayList<>();
        //t, true error, estimate error, open, predicted error, judged in, committed, expected contact - t, ball drop, volts
        final List<double[]> trace = new ArrayList<>();
        int holds, contactsSeen;
        double triggerSeconds, steadySquares, steadyCount, overshootSum, overshootCount;

        int misses() {

            int n = 0;
            for (Ball b : balls) if (b.miss()) n++;
            return n;
        }

        //share of the index-th balls of their bursts that missed (0: every one past the first)
        double missRate(int index) {

            int n = 0, m = 0;
            for (Ball b : balls) {
                if (index == 0 ? b.index < 2 : b.index != index) continue;
                n++;
                if (b.miss()) m++;
            }
            return n > 0 ? (double) m / n : Double.NaN;
        }

        double percentile(int index, double p) {

            List<Double> e = new ArrayList<>();
            for (Ball b : balls) if (index < 0 || b.index == index) e.add(Math.abs(b.error));
            if (e.isEmpty()) return Double.NaN;
            Double[] a = e.toArray(new Double[0]);
            Arrays.sort(a);
            return a[Math.min(a.length - 1, (int) Math.floor(p * a.length))];
        }

        double median(int index) {

            List<Double> e = new ArrayList<>();
            for (Ball b : balls) if (b.index == index) e.add(b.error);
            if (e.isEmpty()) return Double.NaN;
            Double[] a = e.toArray(new Double[0]);
            Arrays.sort(a);
            return a[a.length / 2];
        }

        double secondsPerBall() {
            return balls.isEmpty() ? Double.NaN : triggerSeconds / balls.size();
        }

        double steadyRms() {
            return steadyCount > 0 ? Math.sqrt(steadySquares / steadyCount) : Double.NaN;
        }

        String summary() {
            return String.format(java.util.Locale.ROOT, "%3d balls, %2d miss, |err| p50 %4.1f p95 %4.1f max %5.1f, ball1 %+5.1f ball2 %+5.1f ball3 %+5.1f, %.2f s/ball, %d holds, steady rms %.1f", balls.size(), misses(), percentile(-1, 0.5), percentile(-1, 0.95), percentile(-1, 1.0), median(1), median(2), median(3), secondsPerBall(), holds, steadyRms());
        }
    }

    //the hub, motor and wheel
    static final class Plant implements DcMotorEx {

        final Config c;
        double w, pos, duty, bus, load, openVolts, otherAmps;
        double pending = Double.NaN, pendingAt, readTime;
        double ballLeft, ballRate;
        private final int ring = 400;
        private final double[] times = new double[ring], positions = new double[ring];
        private int newest = -1, count;
        private Direction direction = Direction.FORWARD;
        private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
        private final Random rnd;

        Plant(Config c, Random rnd) {

            this.c = c;
            this.rnd = rnd;
            bus = openVolts = c.startVolts;
        }

        void step(double t) {

            if (pending == pending && t >= pendingAt) {
                duty = pending;
                pending = Double.NaN;
            }
            double u = duty * bus;
            double net = u - c.kv * w - c.ks * Math.signum(w) - load;
            w += c.accel * Math.max(net, -c.coast) * H;
            if (ballLeft > 0) {
                double take = Math.min(ballLeft, ballRate * H);
                w -= take;
                ballLeft -= take;
            }
            pos += w * H;
            double amps = Math.max(0, (u - c.kv * w) / c.rMotor);
            bus = openVolts - c.rInternal * (amps + otherAmps);
            newest = (newest + 1) % ring;
            times[newest] = t;
            positions[newest] = pos;
            count = Math.min(count + 1, ring);
        }

        void ball(double drop) {

            ballLeft += drop;
            ballRate = drop / c.ballSeconds;
        }

        private double positionAt(double t) {

            int k = newest;
            for (int n = 0; n < count; n++) {
                if (times[k] <= t) return positions[k];
                k = (k - 1 + ring) % ring;
            }
            return positions[(newest - count + 1 + ring) % ring];
        }

        @Override
        public double getVelocity() {

            double at = readTime - rnd.nextDouble() * c.readAgeMax;
            return (Math.floor(positionAt(at)) - Math.floor(positionAt(at - WINDOW))) / WINDOW;
        }

        @Override
        public int getCurrentPosition() {
            return (int) Math.floor(positionAt(readTime));
        }

        @Override
        public void setPower(double power) {
            pending = Math.max(-1, Math.min(1, power));
        }

        @Override
        public double getPower() {
            return duty;
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

    static FlywheelPIDVSCoefficients copyCoefficients() {

        FlywheelPIDVSCoefficients k = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        return new FlywheelPIDVSCoefficients(k.kpFar, k.kpClose, k.kiFar, k.kiClose, k.kd, k.kv, k.ks, k.ksSignWidth, k.iSwitch, k.pSwitch, k.kISmash, k.minP, k.maxP, k.minI, k.maxI, k.minD, k.maxD);
    }

    interface Tweak {
        void apply(Flywheel f, FeedGate g);
    }

    //every loop, after the flywheel's update (where the SHT tuner sits)
    interface Hook {
        void loop(double t, Flywheel f, FeedGate g, boolean open);
    }

    static Result run(Config c) {
        return run(c, null, null);
    }

    static Result run(Config c, Tweak tweak, Hook hook) {

        Random rnd = new Random(c.seed);
        Result r = new Result();
        Plant plant = new Plant(c, rnd);
        FeedGate gate = new FeedGate();
        try (SimClock clock = new SimClock()) {
            HardwareMap map = new HardwareMap();
            map.simVoltageSensor = () -> plant.bus + 0.02 * rnd.nextGaussian();
            Flywheel code = new Flywheel(plant);
            code.setVelocityPIDVSCoefficients(copyCoefficients());
            code.initVoltageSensor(map);
            code.useObserver = c.observer;
            if (tweak != null) tweak.apply(code, gate);

            double target = 1460, nextLoop = 0, nextBurst = 3, triggerUntil = -1, burstEnd = -1;
            double driveUntil = -1, nextContact = Double.NaN;
            int burst = 0, ballsLeft = 0, index = 0;
            boolean open = false, wasOpen = false;
            double below = c.below, above = c.above, trueBelow = below, trueAbove = above;
            for (double t = 0; t < c.seconds; t += H) {
                double f = t / c.seconds;
                plant.openVolts = c.startVolts + (c.endVolts - c.startVolts) * f;
                plant.load = c.startLoad + (c.endLoad - c.startLoad) * f;
                if (t >= driveUntil && rnd.nextDouble() < c.drivePerSecond * H) driveUntil = t + 0.2 + 0.4 * rnd.nextDouble();
                plant.otherAmps = 2 + (t < driveUntil ? c.driveAmps * (0.4 + 0.6 * rnd.nextDouble()) : 0);

                if (t >= nextBurst - c.settle && t < nextBurst - c.settle + H) target = 1425 + 85 * rnd.nextDouble();
                if (t >= nextBurst) {
                    burst++;
                    ballsLeft = (int) c.balls;
                    index = 0;
                    triggerUntil = t + 3;
                    burstEnd = Double.NaN;
                    below = c.below * (0.85 + 0.3 * rnd.nextDouble());
                    above = c.above * (0.85 + 0.3 * rnd.nextDouble());
                    trueBelow = below * (1 + c.windowSpread * (2 * rnd.nextDouble() - 1));
                    trueAbove = above * (1 + c.windowSpread * (2 * rnd.nextDouble() - 1));
                    nextBurst = t + c.burstsEvery * (0.7 + 0.6 * rnd.nextDouble());
                }
                boolean trigger = t < triggerUntil && ballsLeft > 0;
                if (trigger) r.triggerSeconds += H;

                if (t >= nextLoop) {
                    double dt = Math.max(0.015, Math.min(0.15, c.loopMedian * Math.exp(c.loopSpread * rnd.nextGaussian())));
                    nextLoop = t + dt;
                    clock.set(t);
                    plant.readTime = t;
                    if (c.gate) {
                        final Flywheel fw = code;
                        open = gate.update(t, trigger, target, fw::predictVelocity, code.expectedContact, code.firstContactSeconds, code.ballIntervalSeconds, below, above);
                    }
                    else open = trigger;
                    code.setFeeding(open);
                    //as the Shooter sets them from CellGeometry
                    code.windowBelow = below;
                    code.windowAbove = above;
                    code.setVelocity(target, false);
                    code.update();
                    if (hook != null) hook.loop(t, code, gate, open);
                    plant.pendingAt = t + 0.3 * dt;
                    if (c.trace) r.trace.add(new double[] {t, plant.w - target, code.getObserverVelocity() - target, open ? 1 : 0, gate.predictedError, gate.judgedIn, gate.committed, code.expectedContact - t, code.observer.ballDrop, plant.bus});
                    double e = plant.w - target;
                    if (!open && ballsLeft == 0 && t - burstEnd > 1 && Math.abs(e) < 60) {
                        r.steadySquares += e * e;
                        r.steadyCount++;
                    }
                }
                plant.step(t);

                //the blocker: a ball reaches the wheel firstContact after it opens, then every interval; shut, the next one waits
                //unless it's already within commit of the wheel
                if (open && !wasOpen && ballsLeft > 0 && !(nextContact == nextContact)) nextContact = t + c.firstContact + 0.015 * rnd.nextGaussian();
                if (!open && wasOpen && nextContact == nextContact && nextContact - t > c.commit) nextContact = Double.NaN;
                wasOpen = open;
                if (nextContact == nextContact && t >= nextContact && ballsLeft > 0) {
                    Ball b = new Ball();
                    b.burst = burst;
                    b.index = ++index;
                    b.time = t;
                    b.error = plant.w - target;
                    b.below = trueBelow;
                    b.above = trueAbove;
                    b.volts = plant.bus;
                    b.shift = c.tableError + (c.ballSpread > 0 ? c.ballSpread * rnd.nextGaussian() : 0);
                    if (c.trace) r.trace.add(new double[] {t, b.error, Double.NaN, -1, b.below, b.above, b.index, 0, 0, 0});
                    r.balls.add(b);
                    plant.ball(Math.max(40, c.ballDrop + c.ballDropSpread * rnd.nextGaussian()));
                    ballsLeft--;
                    nextContact = ballsLeft > 0 && open ? t + Math.max(0.08, c.interval + c.intervalSpread * rnd.nextGaussian()) : Double.NaN;
                    if (ballsLeft == 0) burstEnd = t;
                }
                if (ballsLeft == 0 && burstEnd == burstEnd && t - burstEnd > 0.05 && t - burstEnd < 1.0) {
                    double over = plant.w - target;
                    if (over > 0) {
                        r.overshootSum += over * H;
                        r.overshootCount += H;
                    }
                }
            }
            r.holds = gate.holds;
            r.contactsSeen = code.getContacts();
        }
        return r;
    }
}
