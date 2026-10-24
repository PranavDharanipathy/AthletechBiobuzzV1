package org.athletech.sim.predict;

import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.undeterminables.LoopTimingModel;
import org.athletech.sim.utility.Rng;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Runs the robot's controllers the way FTC code does - once per loop, reading sensors near the
 * start of the loop and commanding motors near the end, with each loop taking a random time -
 * against the continuous physics models. This shows what slow or jittery loops cost: slower turns
 * and moves, overshoot, flywheel speed error at the feed, odometry integration error, and how stale
 * the pose is when it is used.
 *
 * <p>Physics advances in steps of at most {@link #PHYSICS_DT} and lands exactly on every sensor
 * read and motor command. Sensors that update on their own clock (IMU, odometry computer) are
 * sample-and-hold: a read returns their last update.
 */
public final class LoopControlSim {
    private LoopControlSim() {
    }

    public static final double PHYSICS_DT = 0.0005;

    /** Result of a step move (turn or drive). Angles in degrees, distances in meters. */
    public static final class StepResult {
        public final double settleS, overshoot, endError;

        StepResult(double settleS, double overshoot, double endError) {
            this.settleS = settleS;
            this.overshoot = overshoot;
            this.endError = endError;
        }
    }

    public static final class Gains {
        public final double kp, kd;
        public final boolean tuned;

        public Gains(double kp, double kd, boolean tuned) {
            this.kp = kp;
            this.kd = kd;
            this.tuned = tuned;
        }
    }

    /** A sensor with its own update clock: reads return the value latched at its last update. */
    static final class HeldSensor {
        final double period;
        double next, a, b;

        HeldSensor(double hz, Rng rng) {
            this.period = hz > 0 && Double.isFinite(hz) ? 1 / hz : 0;
            this.next = period > 0 ? rng.uniform() * period : Double.POSITIVE_INFINITY;
        }
    }

    // ------------------------------------------------------------------ drive plants

    private static final class DriveRun {
        final MecanumDrive d;
        final Battery battery;
        final double otherA;
        final HeldSensor sensor;
        final boolean turn;
        double t;
        final List<double[]> history = new ArrayList<>();

        DriveRun(MecanumDrive d, Battery battery, double otherA, HeldSensor sensor, boolean turn) {
            this.d = d;
            this.battery = battery;
            this.otherA = otherA;
            this.sensor = sensor;
            this.turn = turn;
        }

        double pos() { return turn ? d.heading() : d.x(); }

        double vel() { return turn ? d.omega() : d.vx(); }

        void advanceTo(double tEnd) {
            while (t < tEnd - 1e-12) {
                double stop = Math.min(tEnd, t + PHYSICS_DT);
                if (sensor.next < stop) stop = Math.max(t + 1e-9, sensor.next);
                d.step(stop - t, battery, otherA);
                t = stop;
                if (t >= sensor.next - 1e-12) {
                    sensor.a = pos();
                    sensor.b = vel();
                    sensor.next += sensor.period;
                }
                history.add(new double[]{t, pos()});
            }
        }
    }

    /**
     * A PD step move: turn by {@code target} radians (turn = true) or drive forward {@code target}
     * meters. The controller reads position/rate from a sensor updating at {@code sensorHz}
     * (Double.POSITIVE_INFINITY or NaN = the true value at the read).
     */
    public static StepResult step(MecanumDrive.Params p, Supplier<Battery> bat, double otherA, LoopTimingModel loop,
                                  double kp, double kd, double sensorHz, boolean turn, double target, double tolerance,
                                  double durationS, Rng rng) {
        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        boolean held = Double.isFinite(sensorHz) && sensorHz > 0;
        HeldSensor s = new HeldSensor(held ? sensorHz : 0, rng);
        DriveRun run = new DriveRun(d, bat.get(), otherA, s, turn);
        double u = 0;
        while (run.t < durationS) {
            LoopTimingModel.Loop l = loop.next(rng);
            double t0 = run.t;
            run.advanceTo(t0 + l.senseMs / 1000);
            double meas = held ? s.a : run.pos(), rate = held ? s.b : run.vel();
            double cmd = kp * (target - meas) - kd * rate;
            u = Math.max(-1, Math.min(1, cmd));
            run.advanceTo(t0 + l.actMs / 1000);
            if (turn) d.drive(0, 0, u);
            else d.drive(u, 0, 0);
            run.advanceTo(t0 + l.periodMs / 1000);
        }
        return metrics(run.history, target, tolerance, durationS);
    }

    /** Settle time, overshoot, and the RMS error over the last 0.5 s (a steady error or a limit cycle). */
    static StepResult metrics(List<double[]> h, double target, double tol, double duration) {
        double lastOutside = 0, over = 0, sq = 0, wsum = 0;
        double sign = Math.signum(target), tailStart = duration - 0.5, prevT = 0;
        for (double[] q : h) {
            double err = q[1] - target;
            if (Math.abs(err) > tol) lastOutside = q[0];
            over = Math.max(over, sign * err);
            double dt = q[0] - prevT;
            if (q[0] > tailStart) {
                sq += err * err * dt;
                wsum += dt;
            }
            prevT = q[0];
        }
        double end = wsum > 0 ? Math.sqrt(sq / wsum) : Double.NaN;
        double settle = lastOutside >= duration - 0.05 ? Double.NaN : lastOutside;
        return new StepResult(settle, over, end);
    }

    /**
     * Grid-searches PD gains on a fast, steady loop (2 ms) with the real sensor update rate: the
     * quickest settle with overshoot under the tolerance. Used when the profile has no gains, so
     * loop-time effects can still be shown.
     */
    public static Gains tune(MecanumDrive.Params p, Supplier<Battery> bat, double otherA, boolean turn, double target, double tol,
                             double sensorHz, Rng rng) {
        LoopTimingModel fast = LoopTimingModel.synthetic(2, 0, 0.3, 0.3);
        double[] kps = turn ? new double[]{0.4, 0.7, 1.0, 1.5, 2.2, 3.0, 4.5, 6.5} : new double[]{0.8, 1.5, 2.5, 4, 6, 9, 13, 18};
        double[] kds = turn ? new double[]{0, 0.03, 0.06, 0.1, 0.15, 0.22, 0.3} : new double[]{0, 0.15, 0.3, 0.5, 0.8, 1.2, 1.7};
        double best = Double.POSITIVE_INFINITY, bestKp = kps[0], bestKd = kds[0];
        double fallback = Double.POSITIVE_INFINITY, fbKp = kps[0], fbKd = kds[0];
        for (double kp : kps) {
            for (double kd : kds) {
                StepResult r = step(p, bat, otherA, fast, kp, kd, sensorHz, turn, target, tol, 2.5, new Rng(7));
                if (!Double.isFinite(r.settleS)) continue;
                if (r.overshoot <= tol && r.settleS < best) {
                    best = r.settleS;
                    bestKp = kp;
                    bestKd = kd;
                }
                if (r.settleS < fallback) {
                    fallback = r.settleS;
                    fbKp = kp;
                    fbKd = kd;
                }
            }
        }
        return Double.isFinite(best) ? new Gains(bestKp, bestKd, true) : new Gains(fbKp, fbKd, true);
    }

    // ------------------------------------------------------------------ flywheel

    /** Speed error (actual - target, rpm) at each feed, and the time from the first feed to the last. */
    public static final class FlywheelResult {
        public final double[] errors;
        public final double burstS;

        FlywheelResult(double[] errors, double burstS) {
            this.errors = errors;
            this.burstS = burstS;
        }
    }

    /**
     * Spins the flywheel up, then fires {@code balls} balls as fast as the feeder and the ready
     * check allow. The ready check runs once per loop (feeds start at the loop's motor-command
     * time). The velocity controller runs in your loop, or on the hub at {@code hubHz} when
     * {@code onHub}. Returns the speed error (actual - target, rpm) at each feed.
     */
    public static FlywheelResult flywheelFeedErrors(Flywheel.Params fp, double targetRpm, double readyTolRpm, double feedIntervalS,
                                              int balls, double ballMassKg, double exitRatio, double lossFactor,
                                              Supplier<Battery> bat, double otherA, LoopTimingModel loop,
                                              boolean onHub, double hubHz, Rng rng) {
        Flywheel f = new Flywheel(fp);
        f.setTargetRpm(targetRpm);
        Battery b = bat.get();
        double kp = fp.kpPerRpm();
        double hubPeriod = onHub && hubHz > 0 ? 1 / hubHz : 0;
        double nextHub = hubPeriod > 0 ? rng.uniform() * hubPeriod : Double.POSITIVE_INFINITY;
        double[] t = {0};
        // advance physics; the hub controller (if used) updates on its own clock
        java.util.function.DoubleConsumer advance = tEnd -> {
            while (t[0] < tEnd - 1e-12) {
                double stop = Math.min(tEnd, t[0] + PHYSICS_DT);
                f.step(stop - t[0], b, otherA);
                t[0] = stop;
            }
        };
        double[] errors = new double[balls];
        int fired = 0;
        double lastFeed = Double.NEGATIVE_INFINITY, waitStart = Double.NaN, firstFeed = Double.NaN;
        boolean spunUp = false;
        double inBand = 0;
        double maxT = 6 + balls * 3.0;
        f.setManualDuty(0);
        while (t[0] < maxT && fired < balls) {
            LoopTimingModel.Loop l = loop.next(rng);
            double t0 = t[0], tEnd = t0 + l.periodMs / 1000;
            // hub control updates inside this loop
            if (onHub) {
                while (nextHub < tEnd) {
                    advance.accept(nextHub);
                    f.setManualDuty(Math.max(0, Math.min(1, f.feedforwardDuty(f.busVolts() > 0 ? f.busVolts() : b.lastVolts())
                            + kp * (targetRpm - f.rpm()))));
                    nextHub += hubPeriod;
                }
            }
            advance.accept(t0 + l.senseMs / 1000);
            double measured = f.rpm();
            boolean ready = Math.abs(measured - targetRpm) <= readyTolRpm;
            if (!spunUp) {
                inBand = ready ? inBand + l.periodMs / 1000 : 0;
                if (inBand >= 0.3) spunUp = true;
            }
            double duty = Math.max(0, Math.min(1, f.feedforwardDuty(f.busVolts() > 0 ? f.busVolts() : b.lastVolts()) + kp * (targetRpm - measured)));
            advance.accept(t0 + l.actMs / 1000);
            if (!onHub) f.setManualDuty(duty);
            if (spunUp) {
                if (Double.isNaN(waitStart)) waitStart = t[0];
                boolean timeUp = t[0] - waitStart > 3;
                if ((ready || timeUp) && t[0] - lastFeed >= feedIntervalS) {
                    if (fired == 0) firstFeed = t[0];
                    errors[fired++] = f.rpm() - targetRpm;
                    f.fire(ballMassKg, exitRatio, lossFactor);
                    lastFeed = t[0];
                    waitStart = Double.NaN;
                }
            }
            advance.accept(tEnd);
        }
        return new FlywheelResult(fired == balls ? errors : java.util.Arrays.copyOf(errors, fired), fired > 1 ? lastFeed - firstFeed : Double.NaN);
    }

    // ------------------------------------------------------------------ localization

    /**
     * Pose error per meter driven when your loop integrates odometry once per loop on a wiggly path
     * (1 m/s, turn rate swinging +/- 2 rad/s). {@code poseExponential}: arc integration, else
     * straight-line steps.
     */
    public static double odometryErrorPerM(LoopTimingModel loop, boolean poseExponential, double seconds, Rng rng) {
        final double v = 1.0;
        double phase = rng.uniform() * 2 * Math.PI;
        double x = 0, y = 0, h = 0;          // truth
        double xr = 0, yr = 0;               // truth at the last read
        double ex = 0, ey = 0, eh = 0;       // estimate
        double t = 0, sAt = 0, hAt = 0, s = 0;
        double fine = 0.0001;
        while (t < seconds) {
            LoopTimingModel.Loop l = loop.next(rng);
            double tEnd = t + l.periodMs / 1000;
            double tSense = t + l.senseMs / 1000;
            // truth up to the sensor read
            while (t < tSense - 1e-12) {
                double dt = Math.min(fine, tSense - t);
                double w = 2.0 * Math.sin(2 * Math.PI * (t + dt / 2) / 1.7 + phase);
                double mid = h + w * dt / 2;
                x += v * dt * Math.cos(mid);
                y += v * dt * Math.sin(mid);
                h += w * dt;
                s += v * dt;
                t += dt;
            }
            double ds = s - sAt, dh = h - hAt;
            if (poseExponential && Math.abs(dh) > 1e-9) {
                ex += ds / dh * (Math.sin(eh + dh) - Math.sin(eh));
                ey += ds / dh * (Math.cos(eh) - Math.cos(eh + dh));
            } else {
                ex += ds * Math.cos(eh);
                ey += ds * Math.sin(eh);
            }
            eh += dh;
            sAt = s;
            hAt = h;
            xr = x;
            yr = y;
            while (t < tEnd - 1e-12) {
                double dt = Math.min(fine, tEnd - t);
                double w = 2.0 * Math.sin(2 * Math.PI * (t + dt / 2) / 1.7 + phase);
                double mid = h + w * dt / 2;
                x += v * dt * Math.cos(mid);
                y += v * dt * Math.sin(mid);
                h += w * dt;
                s += v * dt;
                t += dt;
            }
        }
        // compare at the last sensor read (the time the estimate is for)
        return sAt > 0 ? Math.hypot(ex - xr, ey - yr) / sAt : 0;
    }

    /**
     * Average age of the pose (ms) at the moment motors act on it: read-to-command delay in the
     * loop plus, for an odometry computer, how old its last update was at the read.
     */
    public static double poseLagMs(LoopTimingModel loop, boolean onDevice, double deviceHz, Rng rng, int n) {
        double sum = 0;
        double devPeriodMs = onDevice && deviceHz > 0 ? 1000 / deviceHz : 0;
        for (int i = 0; i < n; i++) {
            LoopTimingModel.Loop l = loop.next(rng);
            sum += (l.actMs - l.senseMs) + rng.uniform() * devPeriodMs;
        }
        return sum / n;
    }
}
