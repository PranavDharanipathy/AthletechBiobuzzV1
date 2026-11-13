package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.robot.CrServoTurret;
import org.athletech.sim.utility.Args;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
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
    public void matchAimNeverRunsPastTheTravel() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        s.plant = TurretPlants.params(1, 1);
        for (double spin : TurretScenario.MATCH_SPINS) {
            TurretScenario.MatchSpin m = TurretScenario.matchSpin(s, TurretScenario.controller(s, null), spin, 30, 4);
            assertTrue(spin + " rad/s travel " + m.maxTravelDeg, m.maxTravelDeg < ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES);
            assertTrue(spin + " rad/s swings " + m.swings, m.swings >= 2);
        }
    }

    @Test
    public void matchAimUsesTheWholeTravel() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        s.plant = TurretPlants.params(1, 1);
        for (double spin : new double[] {1.5, 3}) {
            TurretScenario.MatchSpin m = TurretScenario.matchSpin(s, TurretScenario.controller(s, null), spin, 30, 4);
            assertTrue(spin + " rad/s first swing at " + m.firstSwingDeg, m.firstSwingDeg > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES - 10);
        }
    }

    @Test
    public void autotunerStopsOnAJammedTurret() {

        TurretScenario.Setup s = TurretScenario.setup(new RobotParams(db), ctx);
        CrServoTurret jammed = new CrServoTurret(s.plant, 1);
        jammed.jammed = true;
        TurretAutotuneEngine e = TurretScenario.autotune(s, jammed, 60, 2);
        assertTrue(e.isFaulted());
        assertTrue(Math.abs(jammed.position()) / s.tpd < 0.1);
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

    //balls hit 0.24 s after the blocker opens, then every 0.34 s (match logs): the flywheel learns that, and a push just before each
    //one makes its dip smaller
    @Test
    public void flywheelExpectsTheBall() {

        FlywheelCodeScenario.Setup s = FlywheelCodeScenario.setup(new RobotParams(db), ctx);
        //the example ball is light (~11 ticks/s a hit): about the robot's real dip instead
        s.ballMass *= 5;
        double without = dip(s, 0), with = dip(s, 2);
        System.out.printf(java.util.Locale.ROOT, "FLYWHEEL dip after each ball: %.1f ticks/s without the pre-kick, %.1f with 2 V%n", without, with);
        assertTrue("pre-kick helps: " + with + " vs " + without, with < without);
    }

    static double dip(FlywheelCodeScenario.Setup s, double preKick) {

        double target = ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY, sum = 0;
        int n = 0;
        try (FlywheelCodeScenario.Rig rig = new FlywheelCodeScenario.Rig(s, s.batteryVolts, 3)) {
            rig.code.setVelocity(target, true);
            rig.code.preKickVolts = preKick;
            rig.code.firstContactSeconds = 0.4;
            rig.code.ballIntervalSeconds = 0.5;
            while (rig.t < 3) rig.tick();
            for (int burst = 0; burst < 4; burst++) {
                double open = rig.t, watchUntil = -1, low = Double.NaN;
                double[] contacts = {open + 0.24, open + 0.58, open + 0.92, open + 1.26};
                int next = 0;
                while (rig.t < open + 1.6) {
                    if (rig.tick()) rig.code.setFeeding(rig.t < open + 1.4);
                    if (rig.t < watchUntil) low = Math.min(low, rig.ticksPerSecond());
                    else if (low == low) {
                        if (burst > 0) {
                            sum += target - low;
                            n++;
                        }
                        low = Double.NaN;
                    }
                    if (next < contacts.length && rig.t >= contacts[next]) {
                        rig.fire();
                        next++;
                        watchUntil = rig.t + 0.15;
                        low = rig.ticksPerSecond();
                    }
                }
                while (rig.t < open + 4) {
                    if (rig.tick()) rig.code.setFeeding(false);
                }
            }
            assertEquals("learned first contact", 0.24, rig.code.firstContactSeconds, 0.05);
            assertEquals("learned ball interval", 0.34, rig.code.ballIntervalSeconds, 0.06);
        }
        return sum / n;
    }
}
