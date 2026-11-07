package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.DriveController;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

//the robot's DriveController and Drivetrain Autotuner against the sim mecanum drive, shoved around
public class DriveCodeTest {

    static ProfileDb db;
    static RobotParams rp;
    static MecanumDrive.Params p;
    static DriveCodeScenario.Tune tune;
    static final double LOOP = 0.03;

    static final DriveCodeScenario.Sticks STILL = (t, o) -> {
        o[0] = o[1] = o[2] = 0;
    };

    @BeforeClass
    public static void open() throws Exception {

        Path dir = Files.createTempDirectory("drive-code-test");
        db = ProfileDb.open(dir.resolve("p.db"));
        db.fillExamples();
        rp = new RobotParams(db);
        p = rp.drive();
        tune = DriveCodeScenario.autotune(p, battery(), LOOP, new DriveCodeScenario.Noise(), 9, 300);
    }

    @AfterClass
    public static void close() throws Exception {
        db.close();
    }

    static Battery battery() {
        return rp.battery(12.8);
    }

    static DriveCodeScenario.Shove shove(final double fx, final double fy, final double torque) {

        return (t, o) -> {
            boolean on = t > 1 && t < 2;
            o[0] = on ? fx : 0;
            o[1] = on ? fy : 0;
            o[2] = on ? torque : 0;
        };
    }

    static double maxTurn(DriveCodeScenario.Trace tr) {

        double m = 0;
        for (double[] r : tr.rows) m = Math.max(m, Math.abs(r[3]));
        return m;
    }

    static double farthest(DriveCodeScenario.Trace tr) {

        double m = -1e9;
        for (double[] r : tr.rows) m = Math.max(m, r[1]);
        return m;
    }

    @Test
    public void autotunerMeasuresTheDriveAndStaysInItsSpace() {

        assertTrue(tune.engine.getStatus(), tune.engine.isDone());
        DriveCodeScenario.DriveModel truth = DriveCodeScenario.DriveModel.identify(p, battery(), LOOP);
        DriveController c = tune.engine.getController();
        for (int a = 0; a < 3; a++) {
            assertEquals("axis " + a + " kV", truth.kV[a], c.kV[a], 0.1 * truth.kV[a]);
            assertTrue("axis " + a + " bandwidth " + c.bandwidth[a], c.bandwidth[a] >= 3);
        }
        assertTrue("travel " + tune.maxTravel, tune.maxTravel < 40);
    }

    //a 60 N defender for a second: the old drive got shoved 17-18 in and stayed there
    @Test
    public void holdPushesBackAgainstShovesInEveryAxis() {

        DriveController c = tune.engine.getController();
        DriveCodeScenario.Noise n = new DriveCodeScenario.Noise();
        double open = DriveCodeScenario.maxDisplacement(DriveCodeScenario.run(p, battery(), null, STILL, shove(0, 60, 0), 4, LOOP, n, 1), 1, 4);
        DriveCodeScenario.Trace side = DriveCodeScenario.run(p, battery(), c, STILL, shove(0, 60, 0), 4, LOOP, n, 1);
        DriveCodeScenario.Trace back = DriveCodeScenario.run(p, battery(), c, STILL, shove(60, 0, 0), 4, LOOP, n, 2);
        assertTrue("side " + DriveCodeScenario.maxDisplacement(side, 1, 4) + " vs open " + open, DriveCodeScenario.maxDisplacement(side, 1, 4) < 0.2 * open);
        assertTrue("back " + DriveCodeScenario.maxDisplacement(back, 1, 4) + " vs open " + open, DriveCodeScenario.maxDisplacement(back, 1, 4) < 0.2 * open);
        double[] end = side.at(4);
        assertTrue("came back to " + Math.hypot(end[1], end[2]), Math.hypot(end[1], end[2]) < 1);
        double openTwist = maxTurn(DriveCodeScenario.run(p, battery(), null, STILL, shove(0, 0, 6), 4, LOOP, n, 4));
        double twist = maxTurn(DriveCodeScenario.run(p, battery(), c, STILL, shove(0, 0, 6), 4, LOOP, n, 4));
        assertTrue("twist " + Math.toDegrees(twist) + " vs open " + Math.toDegrees(openTwist), twist < 0.4 * openTwist);
    }

    //driving slowly, a robot behind pushes: it keeps the stick's speed instead of being run faster (old drive: 18 -> 40 in/s)
    @Test
    public void slowDriveKeepsItsSpeedWhenPushedFromBehind() {

        DriveCodeScenario.Trace tr = DriveCodeScenario.run(p, battery(), tune.engine.getController(), (t, o) -> {
            o[0] = 0.3;
            o[1] = o[2] = 0;
        }, (t, o) -> {
            o[0] = t > 2 && t < 3.5 ? 60 : 0;
            o[1] = o[2] = 0;
        }, 4.5, LOOP, new DriveCodeScenario.Noise(), 3);
        double before = DriveCodeScenario.meanForward(tr, 1.2, 2), during = DriveCodeScenario.meanForward(tr, 2.2, 3.5);
        assertEquals("pushed " + during + " vs " + before, before, during, 0.15 * before);
    }

    //letting go at full speed: stops at least as well as the old drive and doesn't get pulled back afterwards
    @Test
    public void stopsLikeTheOldDriveAndNeverRollsBack() {

        DriveCodeScenario.Sticks full = (t, o) -> {
            o[0] = t < 1.5 ? 1 : 0;
            o[1] = o[2] = 0;
        };
        DriveCodeScenario.Shove none = (t, o) -> {
            o[0] = o[1] = o[2] = 0;
        };
        DriveCodeScenario.Trace open = DriveCodeScenario.run(p, battery(), null, full, none, 3.5, LOOP, new DriveCodeScenario.Noise(), 5);
        DriveCodeScenario.Trace held = DriveCodeScenario.run(p, battery(), tune.engine.getController(), full, none, 3.5, LOOP, new DriveCodeScenario.Noise(), 5);
        double openStop = farthest(open) - open.at(1.5)[1], stop = farthest(held) - held.at(1.5)[1];
        assertTrue("stop " + stop + " vs open " + openStop, stop < openStop + 1);
        assertTrue("rolled back " + (farthest(held) - held.at(3.5)[1]), farthest(held) - held.at(3.5)[1] < 0.5);
    }

    //the Shooter relocalizes (setPose): the pose jumps 20 in, the robot must not lunge after where it used to be
    @Test
    public void relocalizingDoesNotMakeItLunge() {

        DriveCodeScenario.Noise n = new DriveCodeScenario.Noise();
        n.jumpAt = 1;
        n.jumpX = 20;
        DriveCodeScenario.Trace tr = DriveCodeScenario.run(p, battery(), tune.engine.getController(), STILL, shove(0, 0, 0), 4, LOOP, n, 6);
        assertTrue("moved " + DriveCodeScenario.maxDisplacement(tr, 0.5, 4), DriveCodeScenario.maxDisplacement(tr, 0.5, 4) < 0.5);
    }

    //driving into something it can't move, then it gets let go: the leash keeps it from lunging to catch up
    @Test
    public void pinnedThenReleasedDoesNotLunge() {

        DriveCodeScenario.Trace tr = DriveCodeScenario.run(p, battery(), tune.engine.getController(), (t, o) -> {
            o[0] = 0.4;
            o[1] = o[2] = 0;
        }, (t, o) -> {
            o[0] = t > 1 && t < 2.5 ? -400 : 0;
            o[1] = o[2] = 0;
        }, 4.5, LOOP, new DriveCodeScenario.Noise(), 7);
        double cruise = DriveCodeScenario.meanForward(tr, 0.6, 1), peak = 0;
        for (double[] r : tr.rows) if (r[0] > 2.5) peak = Math.max(peak, r[7]);
        assertTrue("after the pin " + peak + " vs cruise " + cruise, peak < 1.3 * cruise);
    }

    //worse Pinpoint and loop timing: the autotuner picks a softer hold that still beats the old drive, with no twitching
    @Test
    public void noisySensingGetsASofterHoldThatStillWorks() {

        DriveCodeScenario.Noise bad = new DriveCodeScenario.Noise().scaled(4, 0.03);
        DriveCodeScenario.Tune rough = DriveCodeScenario.autotune(p, battery(), 0.04, bad, 11, 300);
        assertTrue(rough.engine.getStatus(), rough.engine.isDone());
        DriveController c = rough.engine.getController();
        assertTrue("bandwidth " + c.bandwidth[0] + " vs clean " + tune.engine.getController().bandwidth[0], c.bandwidth[0] <= tune.engine.getController().bandwidth[0]);
        double open = DriveCodeScenario.maxDisplacement(DriveCodeScenario.run(p, battery(), null, STILL, shove(0, 60, 0), 4, 0.04, bad, 1), 1, 4);
        double side = DriveCodeScenario.maxDisplacement(DriveCodeScenario.run(p, battery(), c, STILL, shove(0, 60, 0), 4, 0.04, bad, 1), 1, 4);
        assertTrue("side " + side + " vs open " + open, side < 0.4 * open);
        double wander = DriveCodeScenario.maxDisplacement(DriveCodeScenario.run(p, battery(), c, STILL, shove(0, 0, 0), 4, 0.04, bad, 2), 0.5, 4);
        assertTrue("idle wander " + wander, wander < 0.3);
    }
}
