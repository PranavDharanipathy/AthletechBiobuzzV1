package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.CellGeometry;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.FlywheelHyperTuner;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class FlywheelBurstTest {

    static FlywheelBursts.Config base(boolean observer, boolean gate) {

        FlywheelBursts.Config c = new FlywheelBursts.Config();
        c.observer = observer;
        c.gate = gate;
        return c;
    }

    static void print(String name, FlywheelBursts.Result r) {
        System.out.println(String.format("%-34s %s", name, r.summary()));
    }

    interface Change {
        void apply(FlywheelBursts.Config c);
    }

    //the real wheel off from what the code assumes, every way the logs leave room for
    static final String[] CASES = {"as modeled", "accel 120", "accel 210", "coast 0.4", "coast 1.1", "read age 25 ms", "balls 130", "balls 70", "loops 50 ms", "battery 12->9.8", "drive sags x2", "balls 0.2 s apart", "kv +5%", "kv -5%", "shoot 0.3 s after moving", "0.2 s apart, 10 V", "balls 150", "battery 10 V", "10 V + sags + 130"};
    static final Change[] CHANGES = {
            c -> {},
            c -> c.accel = 120,
            c -> c.accel = 210,
            c -> c.coast = 0.4,
            c -> c.coast = 1.1,
            c -> c.readAgeMax = 0.025,
            c -> c.ballDrop = 130,
            c -> c.ballDrop = 70,
            c -> c.loopMedian = 0.05,
            c -> {
                c.startVolts = 12.0;
                c.endVolts = 9.8;
            },
            c -> {
                c.driveAmps = 45;
                c.drivePerSecond = 1.0;
            },
            c -> c.interval = 0.2,
            c -> c.kv = 0.00475 * 1.05,
            c -> c.kv = 0.00475 * 0.95,
            c -> c.settle = 0.3,
            c -> {
                c.interval = 0.2;
                c.startVolts = 10.5;
                c.endVolts = 9.8;
            },
            c -> c.ballDrop = 150,
            c -> {
                c.startVolts = 10.3;
                c.endVolts = 9.6;
            },
            c -> {
                c.startVolts = 10.3;
                c.endVolts = 9.6;
                c.driveAmps = 45;
                c.drivePerSecond = 1.0;
                c.ballDrop = 130;
            }
    };

    @Test
    public void report() {

        for (int k = 0; k < CASES.length; k++) {
            for (int mode = 0; mode < 3; mode++) {
                FlywheelBursts.Config c = base(mode > 0, mode == 2);
                CHANGES[k].apply(c);
                print(String.format("%-18s %s", CASES[k], mode == 0 ? "filter" : mode == 1 ? "observer" : "obs+gate"), FlywheelBursts.run(c));
            }
        }
    }

    //exit gain -> the speed that hits the aim point at 47.5 in, from `shot_video.py fit` on the 2028-05-15 video (all of them fit it)
    static final double[] GAINS = {0.4, 0.6, 0.8, 1.0};
    static final double[] AIM_SPEED = {1468.3, 1460.1, 1451.3, 1443.1};
    static final double OLD_TABLE = 1471.1, NEW_TABLE = 1463.1;
    //what the robot ran in the video
    static final FlywheelBursts.Tweak OLD_GAINS = (f, g) -> {
        f.getCoefficients().kpClose = 0.072;
        f.kd = 0.0003;
        f.burstLiftShare = 0;
    };

    static FlywheelBursts.Config atGain(int k, boolean observer, double table) {

        //the balls, battery and loops of that same session's log (tools/flywheel_fit.py)
        FlywheelBursts.Config c = FlywheelBursts.Config.fitted();
        c.observer = observer;
        c.below = 0.045 * GAINS[k] * 1460;
        c.above = 0.032 * GAINS[k] * 1460;
        c.tableError = table - AIM_SPEED[k];
        c.ballSpread = 6;
        c.seconds = 600;
        return c;
    }

    //the old controller and table against what the video saw: 8 of 41 first balls rim/long, 5 of 7 later (logged) balls out, on that
    //session's balls (0.17 s apart). Later balls miss far more than first ones at every gain, as in the video. The new system misses
    //less everywhere, but not 0: balls that close can't all be won back
    @Test
    public void oldSystemMissesLikeTheVideo() {

        for (double spread : new double[] {6, 14}) {
            for (int k = 0; k < GAINS.length; k++) {
                FlywheelBursts.Config o = atGain(k, false, OLD_TABLE), n = atGain(k, true, NEW_TABLE);
                o.ballSpread = n.ballSpread = spread;
                FlywheelBursts.Result old = FlywheelBursts.run(o, OLD_GAINS, null), now = FlywheelBursts.run(n);
                double oldRate = old.misses() / (double) old.balls.size(), newRate = now.misses() / (double) now.balls.size();
                System.out.println(String.format(java.util.Locale.ROOT, "spread %2.0f, gain %.1f (window -%.0f/+%.0f): old system first balls %3.0f%% out, later %3.0f%% | new system %4.1f%% out (first %.1f%%, later %.1f%%)", spread, GAINS[k], o.below, o.above, 100 * old.missRate(1), 100 * old.missRate(0), 100 * newRate, 100 * now.missRate(1), 100 * now.missRate(0)));
                assertTrue("later balls " + old.missRate(0) + " first " + old.missRate(1), old.missRate(0) > old.missRate(1) + 0.1);
                assertTrue("new " + newRate + " old " + oldRate, newRate < 0.85 * oldRate);
            }
        }
        System.out.println("video: first balls 20% out (8 of 41, half of them low-confidence rims), later logged balls 71% out (5 of 7)");
    }

    @Test
    public void shtTunerFixesABadModelInOneSession() {

        FlywheelBursts.Config c = base(true, true);
        c.seconds = 900;
        c.burstsEvery = 5;
        final FlywheelPIDVSCoefficients[] k = new FlywheelPIDVSCoefficients[1];
        final FlywheelHyperTuner[] h = new FlywheelHyperTuner[1];
        final int[] seen = new int[2];
        final double[] start = new double[2], heldAt = new double[3];
        FlywheelBursts.Result r = FlywheelBursts.run(c, (f, g) -> {
            f.observer.accelPerVolt = 240;
            f.observer.lag = 0;
            f.getCoefficients().kpClose = 0.015;
            k[0] = f.getCoefficients();
            h[0] = new FlywheelHyperTuner(k[0], f.observer.gain, f.observer.lag, f.observer.accelPerVolt, f.observer.loadGain);
            start[0] = h[0].shots.value("kp_close");
            start[1] = h[0].shots.value("observer_lag");
        }, (t, f, g, open) -> {
            h[0].held(g.heldSeconds);
            heldAt[Math.min(2, (int) (3 * t / c.seconds))] = g.heldSeconds;
            h[0].update(t, f.getTargetVelocity(), f.getCurrentVelocity(), f.getRawVelocity(), f.isFeeding());
            if (f.getContacts() != seen[0]) {
                seen[0] = f.getContacts();
                h[0].contact(f.lastContactError, f.windowBelow, f.windowAbove);
            }
            if (g.starved != seen[1]) {
                seen[1] = g.starved;
                h[0].starved();
            }
            h[0].apply(f, k[0]);
        });
        FlywheelHyperTuner tuner = h[0];
        int n = r.balls.size(), third = n / 3;
        double early = 0, late = 0;
        for (int b = 0; b < third; b++) early += Math.abs(r.balls.get(b).error);
        for (int b = n - third; b < n; b++) late += Math.abs(r.balls.get(b).error);
        int[] perThird = new int[3];
        for (FlywheelBursts.Ball b : r.balls) perThird[Math.min(2, (int) (3 * b.time / c.seconds))]++;
        System.out.println(String.format("SHT sim thirds: balls %d / %d / %d, gate held %.0f / %.0f / %.0f s", perThird[0], perThird[1], perThird[2], heldAt[0], heldAt[1] - heldAt[0], heldAt[2] - heldAt[1]));
        System.out.println(String.format("SHT sim: %d shot trials, %d kept; kp_close %.4f -> %.4f, accel %.0f, lag %.3f -> %.3f; |ball error| first third %.1f, last third %.1f; %d misses (%d in the last third); %d contacts seen of %d balls, %d presses starved, %.2f s/ball", tuner.shots.trials, tuner.shots.keeps, start[0], tuner.shots.knob("kp_close").value, tuner.shots.knob("accel_per_volt").value, start[1], tuner.shots.knob("observer_lag").value, early / third, late / third, r.misses(), lateMisses(r), tuner.contacts, n, tuner.starvedBursts, r.secondsPerBall()));
        //with the gate a bad model costs waiting, not misses: every ball still goes, few miss, and the tuner cuts the wait
        assertTrue("trials " + tuner.shots.trials, tuner.shots.trials >= 5);
        assertTrue("balls " + n, n >= 0.95 * 3 * c.seconds / c.burstsEvery);
        assertTrue("misses " + r.misses(), r.misses() <= 0.01 * n);
        assertTrue("late misses " + lateMisses(r), lateMisses(r) <= 1);
        //from this start its gains move within the noise (the waiting swings +-20% a third); what matters is it never starves
        assertTrue("s/ball " + r.secondsPerBall(), r.secondsPerBall() < 0.5);
        assertTrue("contacts " + tuner.contacts + " balls " + n, Math.abs(tuner.contacts - n) <= 0.1 * n);
    }

    static int lateMisses(FlywheelBursts.Result r) {

        int n = 0;
        for (int b = 2 * r.balls.size() / 3; b < r.balls.size(); b++) if (r.balls.get(b).miss()) n++;
        return n;
    }

    @Test
    public void cellGeometryMatchesTheTool() {

        //python -I tools/field_geometry.py window X Y --goal ... (goals as in FieldConstants)
        CellGeometry g = new CellGeometry();
        double exit = CellGeometry.aimZ() - 48.58;
        assertTrue(g.update(59.25, 56.53, false, 59.25, 20, exit, 72));
        assertEquals(0.054, g.slow, 0.002);
        assertEquals(0.039, g.fast, 0.002);
        assertTrue(g.update(59.25, 87.47, true, 59.25, 130, exit, 72));
        assertEquals(0.049, g.slow, 0.002);
        assertEquals(0.036, g.fast, 0.002);
        assertTrue(g.update(59.25, 87.47, true, 30, 110, exit, 72));
        assertEquals(0.060, g.slow, 0.002);
        assertEquals(0.031, g.fast, 0.002);
        assertEquals(52, g.approachDegrees, 1);
        //too close for a 72 deg arc
        g.update(59.25, 56.53, false, 59.25, 45, exit, 72);
        assertTrue(Double.isNaN(g.slow));
    }
}
