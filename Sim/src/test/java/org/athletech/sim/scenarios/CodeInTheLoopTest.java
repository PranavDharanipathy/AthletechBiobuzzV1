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
}
