package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import org.firstinspires.ftc.teamcode.Utility.MathUtil;

//the wheel's speed now, from the volts sent and the hub's reading. The hub's speed is a 50 ms average read a bit later, so it's
//the speed about `lag` ago; the model carries it to now. The hub can't brake the wheel: with the power cut it only coasts down
//(~100 ticks/s^2, 2028-05-15 logs), so the model never slows it faster than coastVolts' worth
public final class FlywheelObserver {

    public double accelPerVolt, kv, ks, ksSignWidth = 35, coastVolts, lag, gain, loadGain, maxLoad = 1.5, contactDrop;
    public double sameBall = 0.12, reseedError = 150, reseedTime = 0.25;
    //the reading that shows a ball is a 50 ms average, so the ball hit about this much before the time it stands for
    public double contactEarly = 0.02, contactWindow = 0.12;
    //how much speed a ball takes, learned from every contact: what the model couldn't explain in the contactWindow after it
    public double ballDrop = 95;
    //predictions speed it back up this much slower than the model: at full power the battery sags more than the reading shows
    public double predictScale = 0.85;

    public double velocity, load, innovation, lastPast;
    public double contactTime = Double.NaN, contactSpeed = Double.NaN;
    public int contacts, reseeds;
    private double dropSoFar = Double.NaN;

    private static final int HISTORY = 32;
    private static final double STEP = 0.005;
    private final double[] times = new double[HISTORY], speeds = new double[HISTORY];
    private int newest = -1, count;
    private double volts, lastTime = Double.NaN, divergedSince = Double.NaN;

    public void reset() {

        count = 0;
        newest = -1;
        lastTime = Double.NaN;
        divergedSince = Double.NaN;
        velocity = load = innovation = volts = 0;
    }

    //what the motor gets from now to the next update
    public void setVolts(double volts) {
        this.volts = MathUtil.isFinite(volts) ? volts : 0;
    }

    public double getVolts() {
        return volts;
    }

    //feeding: a ball can hit now (blocker open, or just closed with balls still on the way)
    public double update(double now, double measured, boolean feeding) {

        if (!MathUtil.isFinite(measured)) return velocity;
        if (count == 0 || !(now > lastTime) || now - lastTime > 0.3) {
            seed(now, measured);
            return velocity;
        }
        double dt = now - lastTime;
        lastTime = now;
        int steps = (int) Math.ceil(dt / STEP);
        double h = dt / steps;
        for (int k = 0; k < steps; k++) velocity += acceleration(velocity, volts) * h;
        push(now, velocity);

        double past = at(now - lag);
        lastPast = past;
        innovation = measured - past;
        double correction;
        if (feeding && innovation < -contactDrop) {
            //a ball takes speed at once: all of it is speed, none of it load
            correction = innovation;
            if (!(now - lag - contactTime <= sameBall)) {
                learnDrop();
                contacts++;
                contactTime = now - lag - contactEarly;
                contactSpeed = at(contactTime);
                dropSoFar = 0;
            }
        }
        else {
            correction = gain * innovation;
            //the rest of a ball isn't load either
            if (!(dropSoFar == dropSoFar)) load = MathUtil.clamp(load - loadGain * innovation, -maxLoad, maxLoad);
        }
        velocity += correction;
        for (int k = 0; k < count; k++) speeds[k] += correction;
        if (dropSoFar == dropSoFar) {
            if (now - lag - contactTime <= contactWindow) dropSoFar -= correction;
            else learnDrop();
        }

        //model badly off (encoder unplugged, a jam): start over from the reading
        if (Math.abs(innovation) > reseedError && !feeding) {
            if (Double.isNaN(divergedSince)) divergedSince = now;
            else if (now - divergedSince > reseedTime) {
                reseeds++;
                seed(now, measured);
            }
        }
        else divergedSince = Double.NaN;
        return velocity;
    }

    private void learnDrop() {

        if (dropSoFar > 20 && dropSoFar < 400) ballDrop += 0.3 * (dropSoFar - ballDrop);
        dropSoFar = Double.NaN;
    }

    public double acceleration(double speed, double volts) {

        double net = volts - kv * speed - ks * MathUtil.smoothSign(speed, ksSignWidth) - load;
        return accelPerVolt * Math.max(net, -coastVolts);
    }

    //speed after `seconds` if the controller (kp on the speed, feedforward, capped at maxVolts) does its job and a ball takes
    //ballDrop at each of the first `balls` times in `hits` (seconds from now)
    public double predict(double target, double seconds, double kp, double maxVolts, double[] hits, int balls) {

        double w = velocity;
        int steps = (int) Math.min(200, Math.ceil(Math.max(0, seconds) / STEP));
        double h = steps > 0 ? seconds / steps : 0;
        double hold = kv * target + ks * MathUtil.smoothSign(target, ksSignWidth) + load;
        int next = 0;
        for (int k = 0; k < steps; k++) {
            while (next < balls && hits[next] <= k * h) {
                w -= ballDrop;
                next++;
            }
            double u = MathUtil.clamp(hold + kp * (target - w), 0, maxVolts);
            double a = acceleration(w, u);
            w += (a > 0 ? predictScale * a : a) * h;
        }
        return w;
    }

    private void seed(double now, double measured) {

        count = 0;
        newest = -1;
        velocity = measured;
        lastTime = now;
        divergedSince = Double.NaN;
        innovation = 0;
        push(now, measured);
    }

    private void push(double time, double speed) {

        newest = (newest + 1) % HISTORY;
        times[newest] = time;
        speeds[newest] = speed;
        count = Math.min(count + 1, HISTORY);
    }

    //the estimate at time t, between stored updates
    private double at(double t) {

        int k = newest;
        for (int n = 1; n < count; n++) {
            int older = (k - 1 + HISTORY) % HISTORY;
            if (times[older] <= t) {
                double span = times[k] - times[older];
                double f = span > 0 ? (t - times[older]) / span : 1;
                return speeds[older] + MathUtil.clamp(f, 0, 1) * (speeds[k] - speeds[older]);
            }
            k = older;
        }
        return speeds[k];
    }
}
