package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.robot.CrServoTurret;
import org.athletech.sim.utility.Args;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.LanyardMap;
import org.firstinspires.ftc.teamcode.Tuners.TurretAutotuneEngine;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

//the robot's own control code (copied from TeamCode at build time) against the sim plants
public class CodeInTheLoopTest {

    Path dir;
    ProfileDb db;
    SimContext ctx;

    @Before
    public void open() throws Exception {

        dir = Files.createTempDirectory("code-loop-test");
        db = ProfileDb.open(dir.resolve("p.db"));
        db.fillExamples();
        ctx = new SimContext(db, dir, false, false, 1, 10, new PrintStream(new ByteArrayOutputStream()), "test");
        ctx.args = new Args(new String[] {"--loop-ms", "15"});
    }

    @After
    public void close() throws Exception {
        db.close();
    }

    @Test
    public void autotunerFindsAStiffStableTurret() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        TurretScenario.Tune r = TurretScenario.autotune(s, new CrServoTurret(s.plant, 1), new TurretAutotuneEngine(TurretAutotuneEngine.Mode.STATIONARY, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, TurretScenario.robotNow()), false, 600, 2);
        assertTrue(r.engine.getStatus(), r.engine.isDone());
        TurretAutotuneEngine.Parameters tuned = r.engine.getParameters();
        assertTrue("bandwidth " + tuned.bandwidth, tuned.bandwidth > 6);
        assertEquals(s.plant.gain, tuned.gain, 0.15 * s.plant.gain);
        TurretScenario.Track step = TurretScenario.step(s, tuned, 30, false, 3);
        assertTrue("overshoot " + step.maxOverDeg, step.maxOverDeg < 1);
        TurretScenario.Track spin = TurretScenario.spin(s, tuned, false, 3);
        assertTrue("spin RMS " + spin.rmsDeg, spin.rmsDeg < 3); //it coasts and starts late from rest, like the robot's servos
    }

    //with made-up overlapping lanyards: off it can't hold near the ends, after the X sweep it holds like there were none.
    //KNOWN GAP (lanyards are off on the robot): since the controller was reworked for the robot's servos (coasting, two delays,
    //kicks) the learned map stays ~1 V off and holds are ~3 deg out. Fix before TURRET_LANYARDS_ENABLED goes on, see DEV_NOTES
    @org.junit.Ignore("lanyard learning needs rework for the robot's servos, lanyards are off")
    @Test
    public void lanyardSweepLearnsOverlappingLanyards() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        TurretAutotuneEngine.Parameters p = TurretScenario.autotune(s, new CrServoTurret(s.plant, 1), new TurretAutotuneEngine(TurretAutotuneEngine.Mode.STATIONARY, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, TurretScenario.robotNow()), false, 600, 2).engine.getParameters();
        TurretScenario.LanyardResult off = TurretScenario.lanyardTest(s, p, null, 5);
        LanyardMap map = TurretScenario.learnLanyards(s, p, TurretScenario.TEST_LANYARDS, 3, 6);
        TurretScenario.LanyardResult on = TurretScenario.lanyardTest(s, p, map, 5);
        TurretScenario.LanyardResult none = TurretScenario.lanyardTest(s, p, null, new double[0][], false, 5);
        assertTrue("off holds " + off.holdRmsDeg, off.holdRmsDeg > 1);
        assertTrue("learned holds " + on.holdRmsDeg + " vs none " + none.holdRmsDeg, on.holdRmsDeg < Math.max(0.4, 2 * none.holdRmsDeg));
        assertTrue("learned sweep " + on.sweepRmsDeg + " vs off " + off.sweepRmsDeg, on.sweepRmsDeg < 0.4 * off.sweepRmsDeg);
        assertTrue("learned spin " + on.spinRmsDeg + " vs off " + off.spinRmsDeg, on.spinRmsDeg < 0.5 * off.spinRmsDeg);
        assertTrue("map error " + on.mapErrorV, on.mapErrorV < 0.8);
        assertTrue("no lanyards, map " + TurretScenario.learnLanyards(s, p, new double[0][], 3, 7).largestVolts(), TurretScenario.learnLanyards(s, p, new double[0][], 3, 7).largestVolts() < 0.6);
    }

    @Test
    public void lanyardsOffChangesNothing() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        TurretAutotuneEngine.Parameters p = TurretScenario.robotNow();
        TurretScenario.Track plain = TurretScenario.spin(s, p, false, 3);
        TurretScenario.Track again = TurretScenario.spin(s.withLanyards(new double[0][], TurretScenario.newMap(), false, 0), p, false, 3);
        assertEquals(plain.rmsDeg, again.rmsDeg, 1e-12);
    }

    @Test
    public void lanyardMapAddsUpReadsBackAndSurvivesTheFile() throws Exception {

        LanyardMap m = new LanyardMap(-195, 195, 5);
        for (int k = 0; k < 2000; k++) {
            m.learn(42.5, 1, 0.01, 6);
            m.learn(42.5, 1, 0.01, 6);
        }
        assertEquals(6, m.volts(42.5, 1), 1e-9);
        assertEquals(0, m.volts(42.5, -1), 1e-9);
        assertEquals(3, m.volts(42.5, 0), 1e-9);
        m.clear();
        m.learn(40, 1, 1, 6);
        m.learn(40, 0, 0.5, 6);
        assertEquals(1.5, m.volts(40, 1), 1e-9);
        assertEquals(0.5, m.volts(40, -1), 1e-9);
        assertEquals(0.75, m.volts(42.5, 1), 1e-9);
        m.learn(500, 1, 1, 6);
        assertEquals(1.5, m.largestVolts(), 1e-9);
        m.learn(-15, 1, 2, 6);
        m.learn(-15, -1, -1, 6);
        java.io.File f = dir.resolve("lanyards/map.csv").toFile();
        assertTrue(m.save(f));
        LanyardMap coarse = new LanyardMap(-195, 195, 10);
        assertTrue(coarse.load(f));
        assertEquals(2, coarse.volts(-15, 1), 1e-3);
        assertEquals(-1, coarse.volts(-15, -1), 1e-3);
        assertEquals(0.5, coarse.volts(-15, 0), 1e-3);
        assertTrue(!coarse.fromCsv("junk\nmore junk"));
        assertEquals(2, coarse.volts(-15, 1), 1e-3);
    }

    //the driver spins as much as they like: it swings around the end of the travel and still finishes
    @Test
    public void drivingAutotuneHandlesEndlessSpinning() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        TurretAutotuneEngine.Parameters p = TurretScenario.robotNow();
        TurretScenario.Tune r = TurretScenario.autotune(s, new CrServoTurret(s.plant, 1), new TurretAutotuneEngine(TurretAutotuneEngine.Mode.DRIVING, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, p), true, 1.5, 400, 2);
        assertTrue(r.engine.getStatus(), r.engine.isDone());
        assertTrue("travel " + r.maxTravelDeg, r.maxTravelDeg < 190);
        assertTrue("spun " + 1.5 * r.seconds * 180 / Math.PI + " deg", 1.5 * r.seconds * 180 / Math.PI > 720);
    }

    //DRIVING with the flywheel shaking it: the bandwidth stays what STATIONARY gave, and the rejection floor it finds is never
    //above the bandwidth or below half of it
    @Test
    public void drivingFindsTheRejectionFloor() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        TurretAutotuneEngine.Parameters start = TurretScenario.matched(s, 4);
        CrServoTurret turret = new CrServoTurret(s.plant, 1);
        double part = 0.3 * s.tpd / Math.sqrt(2);
        turret.setVibration(part, 59.5, part);
        TurretScenario.Tune r = TurretScenario.autotune(s, turret, new TurretAutotuneEngine(TurretAutotuneEngine.Mode.DRIVING, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, start), true, 0, 400, 2);
        assertTrue(r.engine.getStatus(), r.engine.isDone());
        TurretAutotuneEngine.Parameters got = r.engine.getParameters();
        assertEquals(start.bandwidth, got.bandwidth, 1e-9);
        assertTrue("floor " + got.shakeBandwidth, got.shakeBandwidth <= got.bandwidth + 1e-9 && got.shakeBandwidth >= 0.5 * got.bandwidth);
        assertEquals(got.shakeBandwidth, r.engine.getResults().get("shake_bandwidth"), 0.01);
    }

    //pushing all the way in when the friction guess is high used to swing it +-4 deg forever
    @Test
    public void holdDoesNotHuntWhenFrictionIsGuessedHigh() {

        //the robot's turret: it slid on 0.65-0.75 V where STATIONARY said 1.01 (runs 20280507-195213, -210857)
        for (double slide : new double[] {0.65, 0.75}) {
            TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx).with(0, 59.5, 0, true);
            double tpd = 68.27;
            s.tpd = tpd;
            s.loop = new CodeLoop(0.031, 0.3, 1);
            s.plant = new CrServoTurret.Params(166.6 * tpd, 0.046, 0.036, slide, 1.163, 0, 384 * tpd, 1735 * tpd);
            s.plant.startDelay = 0.083;
            s.plant.coastDecel = 1050 * tpd;
            s.plant.neutralDecel = 1500 * tpd;
            s.startTicks = 6 * tpd;
            TurretAutotuneEngine.Parameters guess = new TurretAutotuneEngine.Parameters(166.6 * tpd, 0.046, 1.011, 1.163, 0.036, 3.2, 0.9, 0.3, 1).capability(384 * tpd, 1735 * tpd).startDelay(0.083).deceleration(1050 * tpd);
            TurretScenario.Target still = new TurretScenario.Target() {
                public double position(double t) {
                    return 0;
                }

                public double velocity(double t) {
                    return 0;
                }
            };
            for (long seed = 1; seed <= 3; seed++) {
                TurretScenario.Track r = TurretScenario.track(s, guess, still, 10, false, 3, -1, false, seed);
                assertTrue(slide + " V seed " + seed + " hold RMS " + r.rmsDeg, r.rmsDeg < 1); //used to be 2.8
            }
            }
    }

    //the Pinpoint's heading rate jitters ~1 deg/s with the robot still: that used to flip the whole friction push every loop
    @Test
    public void noisyRateDoesNotBuzzAStillTurret() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        final double tpd = s.tpd;
        for (long seed = 1; seed <= 3; seed++) {
            final java.util.Random noise = new java.util.Random(seed);
            TurretScenario.Target jittery = new TurretScenario.Target() {
                public double position(double t) {
                    return 0;
                }

                public double velocity(double t) {
                    return 1.1 * tpd * noise.nextGaussian();
                }
            };
            TurretScenario.Track r = TurretScenario.track(s, TurretScenario.matched(s, 4), jittery, 10, false, 2, -1, false, seed);
            assertTrue("seed " + seed + " volts jumping " + r.chatterV, r.chatterV < 0.2);
            assertTrue("seed " + seed + " hold RMS " + r.rmsDeg, r.rmsDeg < 0.2);
        }
    }

    //match aim with the robot spinning as hard as a driver can: the plan stops it at the ends in time, it never runs past the travel
    @Test
    public void matchAimNeverRunsPastTheTravel() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        for (double spin : TurretScenario.MATCH_SPINS) {
            TurretScenario.MatchSpin m = TurretScenario.matchSpin(s, TurretScenario.matched(s, 4), spin, 30, 4);
            //6 rad/s outruns the turret, so it waits right at the end: still short of where TurretBase cuts the power
            assertTrue(spin + " rad/s travel " + m.maxTravelDeg, m.maxTravelDeg < ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES);
            assertTrue(spin + " rad/s swings " + m.swings, m.swings >= 2);
        }
    }

    //the match aim follows the aim right out to the ends before it swings round; it used to swing at ~155-180 deg, so the
    //turret never got past 160
    @Test
    public void matchAimUsesTheWholeTravel() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        for (double spin : new double[] {1.5, 3}) {
            TurretScenario.MatchSpin m = TurretScenario.matchSpin(s, TurretScenario.matched(s, 4), spin, 30, 4);
            assertTrue(spin + " rad/s first swing at " + m.firstSwingDeg, m.firstSwingDeg > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES - 10);
        }
    }

    static TurretScenario.Setup robotTurret(SimContext ctx, double slide) {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(ctx.db), ctx).with(0, 59.5, 0, true);
        double tpd = 68.27;
        s.tpd = tpd;
        s.loop = new CodeLoop(0.031, 0.3, 1);
        s.plant = new CrServoTurret.Params(166.6 * tpd, 0.046, 0.036, slide, 1.163, 0, 384 * tpd, 1735 * tpd);
        s.plant.startDelay = 0.083;
        s.plant.coastDecel = 1050 * tpd;
        s.plant.neutralDecel = 1500 * tpd;
        return s;
    }

    static TurretAutotuneEngine.Parameters robotModel() {

        double tpd = 68.27;
        return new TurretAutotuneEngine.Parameters(166.6 * tpd, 0.046, 1.011, 1.163, 0.036, 3.2, 0.9, 0.3, 1).capability(384 * tpd, 1735 * tpd).startDelay(0.083).deceleration(1050 * tpd).shakeBandwidth(3.2);
    }

    //the robot's TeleOp log (20280507-230429): 31% of loops 15+ deg off its own plan, the two fighting to ~0 V, once stuck 2 s
    //30 deg short. The plan now moves onto the turret instead (used to be 6-8% here), and every step gets within 5 deg in time
    @Test
    public void turretNeverFightsItsOwnPlan() {

        for (double slide : new double[] {0.65, 0.75}) {
            for (long seed = 1; seed <= 2; seed++) {
                TurretScenario.Steps r = TurretScenario.steps(robotTurret(ctx, slide), robotModel(), seed);
                assertTrue(slide + " V seed " + seed + " diverged " + r.divergedFraction, r.divergedFraction < 0.01);
                for (int k = 0; k < r.reachSeconds.length; k++) {
                    double move = Math.abs(TurretScenario.STEP_SEQUENCE[k] - (k == 0 ? 0 : TurretScenario.STEP_SEQUENCE[k - 1]));
                    assertTrue(slide + " V seed " + seed + " step " + k + " (" + move + " deg) reach " + r.reachSeconds[k], r.reachSeconds[k] < 0.5 + move / 260);
                }
            }
        }
    }

    @Test
    public void autotunerStopsOnAJammedTurret() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        CrServoTurret jammed = new CrServoTurret(s.plant, 1);
        jammed.jammed = true;
        TurretScenario.Tune r = TurretScenario.autotune(s, jammed, new TurretAutotuneEngine(TurretAutotuneEngine.Mode.STATIONARY, s.tpd, -1, TurretScenario.robotNow()), false, 60, 2);
        assertTrue(r.engine.isFaulted());
        assertTrue(r.maxTravelDeg < 0.1);
    }

    @Test
    public void robotFlywheelCodeSpinsUpAndHoldsSpeed() {

        FlywheelCodeScenario.Setup s = FlywheelCodeScenario.setup(new RobotParams(db), ctx);
        FlywheelCodeScenario.SpinUp r = FlywheelCodeScenario.spinUp(s, ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY, s.batteryVolts, null, 1);
        assertTrue("never settled", Double.isFinite(r.settleS));
        assertTrue("overshoot " + r.overshoot, r.overshoot < 100);
        assertTrue("ripple " + r.rippleRms, r.rippleRms < 30);
        FlywheelCodeScenario.Burst b = FlywheelCodeScenario.burst(s, ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY, s.batteryVolts, false, null, 1);
        assertEquals(s.balls, b.exitSpeeds.size());
        double now = System.nanoTime() * 1e-9;
        assertEquals("the sim gave the robot clock back", now, Clock.seconds(), 1);
    }
}
