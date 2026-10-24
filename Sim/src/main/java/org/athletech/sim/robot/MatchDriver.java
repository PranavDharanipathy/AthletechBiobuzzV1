package org.athletech.sim.robot;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.utility.Rng;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Plays one match with the full drive (and, when available, flywheel) physics to get what the
 * electrical and static sims need: where the robot is and how fast it moves (every 10 ms), the
 * battery voltage and total current (every 1 ms), and every hit - walls at the speeds the driver
 * hits them, pushing against walls and robots with stalled motors, and contacts with robots.
 *
 * <p>Driving: random waypoints with full-power moves and pauses (intaking, shooting, waiting),
 * plus scheduled wall hits and robot contacts at the profile's rates. The robot is disabled
 * between AUTO and TELEOP. The flywheel (if the launcher is in the profile) spins at max_rpm
 * through TELEOP and fires a full load at some pauses.
 */
public final class MatchDriver {
    private MatchDriver() {
    }

    public static final double PHYSICS_DT = 0.001, REC_DT = 0.01;

    /** Driver behaviour and robot build bits the match needs. */
    public static final class Behaviour {
        public double autoS = 30, transitionS = 8, teleopS = 120;
        public double fieldSizeM = 3.6576, lengthM = 0.43, widthM = 0.43;
        public double wallHitsPerMatch = 6, wallPushS = 0.6, robotContactsPerMatch = 4, intakesPerMatch = 20, metalContactsPerMatch = 2;
        public Distribution wallHitSpeed = Distribution.normal("wall", 0.8, 0.3, "m/s");
        public Distribution robotContactSpeed = Distribution.normal("robot", 0.7, 0.3, "m/s");
        public double contactTimeMs = 12, restitution = 0.2;
        public int ballsPerBurst = 3;

        public double matchS() { return autoS + transitionS + teleopS; }

        public boolean enabled(double t) { return t < autoS || t >= autoS + transitionS; }
    }

    /** Everything recorded from one simulated match. */
    public static final class Trace {
        public final double voc;
        public final double[] x, y, h, vf, vl, w;   // every REC_DT
        /** Average wheel slip speed against the tiles over each REC_DT, m/s (pushing, hard starts and stops). */
        public final double[] slip;
        public final float[] busV, current;         // every PHYSICS_DT
        public final List<double[]> wallHits;       // {t, dv m/s, g}
        public final List<double[]> robotContacts;  // {t, closing speed m/s, g}
        public final List<Double> intakes, metalContacts;
        public final double distanceM, slipM;

        Trace(double voc, double[] x, double[] y, double[] h, double[] vf, double[] vl, double[] w, double[] slip, float[] busV, float[] current,
              List<double[]> wallHits, List<double[]> robotContacts, List<Double> intakes, List<Double> metalContacts, double distanceM, double slipM) {
            this.voc = voc;
            this.x = x;
            this.y = y;
            this.h = h;
            this.vf = vf;
            this.vl = vl;
            this.w = w;
            this.slip = slip;
            this.busV = busV;
            this.current = current;
            this.wallHits = wallHits;
            this.robotContacts = robotContacts;
            this.intakes = intakes;
            this.metalContacts = metalContacts;
            this.distanceM = distanceM;
            this.slipM = slipM;
        }

        /** Average battery current over record step {@code k} (REC_DT), A. */
        public double currentAt(int k) {
            int per = (int) Math.round(REC_DT / PHYSICS_DT), i0 = k * per;
            double s = 0;
            int n = 0;
            for (int i = i0; i < Math.min(current.length, i0 + per); i++, n++) s += current[i];
            return n == 0 ? 0 : s / n;
        }

        public double minBusV() {
            double m = Double.POSITIVE_INFINITY;
            for (float v : busV) m = Math.min(m, v);
            return m;
        }

        public double peakCurrent() {
            double m = 0;
            for (float c : current) m = Math.max(m, c);
            return m;
        }

        public double peakImpactG() {
            double m = 0;
            for (double[] e : wallHits) m = Math.max(m, e[2]);
            for (double[] e : robotContacts) m = Math.max(m, e[2]);
            return m;
        }
    }

    private enum Mode { GOTO, PAUSE, RAM, PUSH }

    /**
     * @param lp flywheel launcher, or null to leave it out
     */
    public static Trace simulate(MecanumDrive.Params p, Battery battery, double otherA, Launcher.Params lp, Behaviour b, Rng rng) {
        double matchS = b.matchS(), half = b.fieldSizeM / 2;
        int nRec = (int) Math.ceil(matchS / REC_DT) + 1, nPhys = (int) Math.ceil(matchS / PHYSICS_DT) + 1;
        double[] xs = new double[nRec], ys = new double[nRec], hs = new double[nRec], vfs = new double[nRec], vls = new double[nRec], ws = new double[nRec];
        double[] slips = new double[nRec];
        double slipAcc = 0, slipM = 0;
        int slipN = 0;
        float[] bus = new float[nPhys], cur = new float[nPhys];
        List<double[]> wallHits = new ArrayList<>(), robotContacts = new ArrayList<>();
        double activeS = b.autoS + b.teleopS;

        // scheduled events (Poisson over the enabled time)
        List<Double> ramTimes = poissonTimes(b.wallHitsPerMatch, b, rng);
        List<Double> contactTimes = poissonTimes(b.robotContactsPerMatch, b, rng);
        List<Double> intakes = poissonTimes(b.intakesPerMatch, b, rng);
        List<Double> metal = poissonTimes(b.metalContactsPerMatch, b, rng);

        double voc0 = battery.openCircuitVolts();
        MecanumDrive d = new MecanumDrive(p);
        double margin = 0.5 * Math.hypot(b.lengthM, b.widthM) + 0.05;
        d.reset(rng.uniform(-half + margin, half - margin), -half + margin, Math.PI / 2);
        double vTop = p.theoreticalTopSpeed() * 0.92;

        Flywheel fly = lp == null ? null : new Flywheel(lp.flywheel());
        Launcher.BallFit fit = lp == null ? null : lp.balls().values().iterator().next();
        double flyCurrent = 0;
        int shotsLeft = 0;
        double lastShot = Double.NEGATIVE_INFINITY;

        Mode mode = Mode.GOTO;
        double tx = rng.uniform(-half + margin, half - margin), ty = rng.uniform(-half + margin, half - margin), th = rng.uniform(-Math.PI, Math.PI);
        double modeUntil = Double.POSITIVE_INFINITY, gotoStart = 0;
        double ramVx = 0, ramVy = 0;
        double[] plane = null; // virtual wall while pushing a robot: {px, py, nx, ny}
        int ramIdx = 0, contactIdx = 0;
        double dist = 0;
        double t = 0;
        int k = 0;
        for (int i = 0; i < nPhys; i++, t = i * PHYSICS_DT) {
            boolean on = b.enabled(t);
            // ---- scheduled hits
            if (on && mode != Mode.RAM && mode != Mode.PUSH && ramIdx < ramTimes.size() && t >= ramTimes.get(ramIdx)) {
                ramIdx++;
                double v = Math.max(0.1, Math.min(vTop, b.wallHitSpeed.sample(rng)));
                // toward the nearest wall
                double[][] walls = {{half - d.x(), 1, 0}, {d.x() + half, -1, 0}, {half - d.y(), 0, 1}, {d.y() + half, 0, -1}};
                double[] wsel = walls[0];
                for (double[] wl : walls) if (wl[0] < wsel[0]) wsel = wl;
                ramVx = wsel[1] * v;
                ramVy = wsel[2] * v;
                mode = Mode.RAM;
                modeUntil = t + 4;
            }
            if (on && mode != Mode.PUSH && contactIdx < contactTimes.size() && t >= contactTimes.get(contactIdx)) {
                contactIdx++;
                double rel = Math.max(0.05, b.robotContactSpeed.sample(rng));
                double g = impactG(rel * (1 + b.restitution) / 2, b.contactTimeMs);
                robotContacts.add(new double[]{t, rel, g});
                double sp = d.speed();
                double nx, ny;
                if (sp > 0.05) {
                    nx = -d.vx() / sp;
                    ny = -d.vy() / sp;
                } else {
                    double a = rng.uniform(-Math.PI, Math.PI);
                    nx = Math.cos(a);
                    ny = Math.sin(a);
                }
                plane = new double[]{d.x() - 0.01 * nx, d.y() - 0.01 * ny, nx, ny};
                ramVx = -nx;
                ramVy = -ny;
                mode = Mode.PUSH;
                modeUntil = t + Math.max(0.2, b.wallPushS);
            }

            // ---- command
            double fwd = 0, left = 0, turn = 0;
            if (on) {
                double c = Math.cos(d.heading()), s = Math.sin(d.heading());
                switch (mode) {
                    case GOTO: {
                        double ex = tx - d.x(), ey = ty - d.y(), eh = Physics.wrap(th - d.heading());
                        fwd = 3 * (c * ex + s * ey);
                        left = 3 * (-s * ex + c * ey);
                        turn = 2 * eh;
                        if ((Math.hypot(ex, ey) < 0.05 && Math.abs(eh) < 0.05) || t - gotoStart > 5) {
                            mode = Mode.PAUSE;
                            modeUntil = t + rng.uniform(0.4, 2.5);
                            if (fly != null && t >= b.autoS && rng.uniform() < 0.35) shotsLeft = b.ballsPerBurst;
                        }
                        break;
                    }
                    case PAUSE:
                        if (t >= modeUntil && shotsLeft == 0) {
                            mode = Mode.GOTO;
                            gotoStart = t;
                            tx = rng.uniform(-half + margin, half - margin);
                            ty = rng.uniform(-half + margin, half - margin);
                            th = rng.uniform(-Math.PI, Math.PI);
                        }
                        break;
                    case RAM:
                    case PUSH: {
                        // drive along (ramVx, ramVy) at the hit speed (RAM) or full power (PUSH)
                        double scale = mode == Mode.RAM ? Math.hypot(ramVx, ramVy) / vTop : 1;
                        double ux = ramVx / Math.max(1e-9, Math.hypot(ramVx, ramVy)), uy = ramVy / Math.max(1e-9, Math.hypot(ramVx, ramVy));
                        fwd = scale * (c * ux + s * uy);
                        left = scale * (-s * ux + c * uy) / p.strafeEfficiency();
                        if (t >= modeUntil) {
                            mode = Mode.GOTO;
                            gotoStart = t;
                            plane = null;
                            tx = rng.uniform(-half + margin, half - margin);
                            ty = rng.uniform(-half + margin, half - margin);
                            th = rng.uniform(-Math.PI, Math.PI);
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
            d.drive(clamp(fwd), clamp(left), clamp(turn));

            // ---- flywheel: spins through TELEOP, fires during shooting pauses
            if (fly != null) {
                boolean spin = t >= b.autoS + b.transitionS;
                fly.setTargetRpm(spin ? lp.maxRpm() : 0);
                if (shotsLeft > 0 && spin && t - lastShot >= lp.feedIntervalS() && Math.abs(fly.rpm() - lp.maxRpm()) <= lp.readyToleranceRpm()) {
                    fly.fire(fit.ball().massKg(), fit.exitSpeedRatio(), fit.energyLossFactor());
                    lastShot = t;
                    shotsLeft--;
                }
                if (!spin) shotsLeft = 0;
            }

            double x0 = d.x(), y0 = d.y();
            d.step(PHYSICS_DT, battery, otherA + flyCurrent);
            if (fly != null) {
                fly.stepAt(PHYSICS_DT, d.busVolts());
                flyCurrent = Math.max(0, fly.batteryCurrent());
            }
            dist += Math.hypot(d.x() - x0, d.y() - y0);
            slipAcc += d.slipSpeedMean();
            slipN++;
            slipM += d.slipSpeedMean() * PHYSICS_DT;

            // ---- walls (and the virtual wall of a robot being pushed)
            double hh = d.heading();
            double ex = 0.5 * b.lengthM * Math.abs(Math.cos(hh)) + 0.5 * b.widthM * Math.abs(Math.sin(hh));
            double ey = 0.5 * b.lengthM * Math.abs(Math.sin(hh)) + 0.5 * b.widthM * Math.abs(Math.cos(hh));
            double dv = 0;
            if (d.x() > half - ex) {
                d.moveTo(half - ex, d.y());
                dv = Math.max(dv, hit(d, -1, 0, b));
            } else if (d.x() < -half + ex) {
                d.moveTo(-half + ex, d.y());
                dv = Math.max(dv, hit(d, 1, 0, b));
            }
            if (d.y() > half - ey) {
                d.moveTo(d.x(), half - ey);
                dv = Math.max(dv, hit(d, 0, -1, b));
            } else if (d.y() < -half + ey) {
                d.moveTo(d.x(), -half + ey);
                dv = Math.max(dv, hit(d, 0, 1, b));
            }
            if (dv > 0.1) {
                wallHits.add(new double[]{t, dv, impactG(dv, b.contactTimeMs)});
                if (mode == Mode.RAM) {
                    mode = Mode.PUSH;
                    modeUntil = t + b.wallPushS;
                }
            }
            if (plane != null) {
                double pen = (d.x() - plane[0]) * plane[2] + (d.y() - plane[1]) * plane[3];
                if (pen < 0) {
                    d.moveTo(d.x() - pen * plane[2], d.y() - pen * plane[3]);
                    d.collide(plane[2], plane[3], 0);
                }
            }

            bus[i] = (float) d.busVolts();
            cur[i] = (float) d.batteryCurrent();
            if (t >= k * REC_DT - 1e-9 && k < nRec) {
                xs[k] = d.x();
                ys[k] = d.y();
                hs[k] = d.heading();
                vfs[k] = d.forwardSpeed();
                vls[k] = d.leftSpeed();
                ws[k] = d.omega();
                slips[k] = slipN == 0 ? 0 : slipAcc / slipN;
                slipAcc = 0;
                slipN = 0;
                k++;
            }
        }
        for (; k < nRec; k++) {
            xs[k] = xs[k - 1];
            ys[k] = ys[k - 1];
            hs[k] = hs[k - 1];
        }
        return new Trace(voc0, xs, ys, hs, vfs, vls, ws, slips, bus, cur, wallHits, robotContacts, intakes, metal, dist, slipM);
    }

    /** Collision with restitution only for a real hit (no chatter while pushing). Returns dv. */
    private static double hit(MecanumDrive d, double nx, double ny, Behaviour b) {
        double vn = -(d.vx() * nx + d.vy() * ny);
        return d.collide(nx, ny, vn > 0.15 ? b.restitution : 0);
    }

    /** Peak acceleration (g) of a half-sine impact that changes speed by dv over the contact time. */
    public static double impactG(double dv, double contactMs) {
        return Math.PI / 2 * dv / Math.max(1e-4, contactMs / 1000) / Physics.G;
    }

    private static double clamp(double v) {
        return Math.max(-1, Math.min(1, v));
    }

    /** Event times of a Poisson process with {@code perMatch} expected events over the enabled time. */
    static List<Double> poissonTimes(double perMatch, Behaviour b, Rng rng) {
        List<Double> out = new ArrayList<>();
        double active = b.autoS + b.teleopS;
        if (!(perMatch > 0) || active <= 0) return out;
        double rate = perMatch / active, s = 0;
        while (true) {
            s += -Math.log(1 - rng.uniform()) / rate;
            if (s >= active) break;
            out.add(s < b.autoS ? s : s + b.transitionS);
        }
        Collections.sort(out);
        return out;
    }
}
