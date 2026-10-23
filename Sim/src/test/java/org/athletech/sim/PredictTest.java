package org.athletech.sim;

import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.MotorSpec;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.predict.AimSolver;
import org.athletech.sim.predict.DriveTimePredictor;
import org.athletech.sim.predict.ShotPredictor;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.undeterminables.OdometryErrorModel;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Vec3;
import org.junit.Test;

import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Prediction algorithms. */
public class PredictTest {

    static Launcher launcher() {
        MotorSpec bare = MotorSpec.fromRpm(0.144, 9.2, 6000, 0.25, 12);
        Flywheel.Params fp = new Flywheel.Params(0.0012, 0.048, 1, 2, bare, 0.005, 0.002, false);
        Projectile.Ball pollen = new Projectile.Ball("pollen", 0.02, 0.071, 0.5, 0);
        return new Launcher(new Launcher.Params(fp, 0.35, 0, 0, 30, 65, 4500, 100, 0.25, true, true,
                Collections.singletonMap("pollen", new Launcher.BallFit(pollen, 0.5, 1.6))));
    }

    static final Aperture CELL = Aperture.oriented("cell", new Vec3(0, 0, 1.25), 0, Math.toRadians(20), 0.508, 0.356, false);

    @Test
    public void aimSolutionScoresAndNeedsMoreSpeedFartherAway() {
        Launcher l = launcher();
        AimSolver.Solution near = AimSolver.solve(l, "pollen", CELL, 1.2, 0);
        AimSolver.Solution far = AimSolver.solve(l, "pollen", CELL, 3.0, 0);
        assertTrue(near.reachable() && far.reachable());
        assertTrue(near.nominal().made() && far.nominal().made());
        assertTrue("rpm should rise with distance", far.rpm() > near.rpm());
        assertTrue(near.hoodDeg() >= 30 && near.hoodDeg() <= 65);
        assertTrue(near.speedLow() < near.exitSpeed() && near.exitSpeed() < near.speedHigh());
        assertFalse("20 m is out of reach", AimSolver.solve(l, "pollen", CELL, 20, 0).reachable());
    }

    @Test
    public void shootingTableNeverJumpsArcsBetweenPoints() {
        Launcher l = launcher();
        double[] d = new double[20];
        for (int i = 0; i < d.length; i++) d[i] = 0.5 + 0.2 * i;
        AimSolver.Table t = AimSolver.solveTable(l, "pollen", CELL, d, 0, v -> 0.02 * v, 1.0);
        assertEquals(d.length, t.solutions().length);
        assertEquals("switch points come in pairs", 0, t.switchPoints().size() % 2);
        java.util.List<AimSolver.Solution> lut = t.lookup();
        assertTrue(lut.size() > 5);
        for (int i = 1; i < lut.size(); i++) {
            AimSolver.Solution a = lut.get(i - 1), b = lut.get(i);
            double gap = b.distanceM() - a.distanceM(), jump = Math.abs(b.hoodDeg() - a.hoodDeg());
            assertTrue("distances ascend", gap > 0);
            // a big hood change is only allowed across a switch pair (1 mm apart)
            assertTrue("hood jumps " + jump + " deg between " + a.distanceM() + " and " + b.distanceM(),
                    jump <= AimSolver.ARC_JUMP_DEG || gap < 0.002);
        }
        for (AimSolver.Solution s : t.solutions()) if (s.reachable()) assertTrue(s.nominal().made());
    }

    @Test
    public void noErrorsMeansEveryShotScores() {
        Launcher l = launcher();
        AimSolver.Solution s = AimSolver.solve(l, "pollen", CELL, 2.0, 0);
        Distribution none = Distribution.none("x", "");
        ShotPredictor.Errors zero = new ShotPredictor.Errors(none, none, none, none, none, none, none);
        assertEquals(1.0, ShotPredictor.predict(l, "pollen", CELL, s, 0, Double.NaN, zero, 200, new Rng(1)).probability(), 0);
        ShotPredictor.Errors big = new ShotPredictor.Errors(Distribution.normal("rpm", 0, 400, "rpm"), none, none, none,
                Distribution.normal("aim", 0, 5, "deg"), none, none);
        double p = ShotPredictor.predict(l, "pollen", CELL, s, 0, Double.NaN, big, 2000, new Rng(1)).probability();
        assertTrue("large errors must cost makes: " + p, p < 0.8);
    }

    @Test
    public void moveTimesGrowWithDistanceAndStopOnTarget() {
        MecanumDrive.Params p = PhysicsTest.drive(0.9, 0.04, 0.8);
        double t1 = DriveTimePredictor.timeTo(p, () -> new Battery(13, 0.07, 0), 0.8, DriveTimePredictor.Axis.FORWARD, 1).timeS();
        double t2 = DriveTimePredictor.timeTo(p, () -> new Battery(13, 0.07, 0), 0.8, DriveTimePredictor.Axis.FORWARD, 2).timeS();
        double tStrafe = DriveTimePredictor.timeTo(p, () -> new Battery(13, 0.07, 0), 0.8, DriveTimePredictor.Axis.STRAFE, 1).timeS();
        assertTrue(t1 > 0.5 && t2 > t1 && t2 < 2 * t1);
        assertTrue("strafing is slower", tStrafe > t1);
        double weak = DriveTimePredictor.timeTo(p, () -> new Battery(11.8, 0.07, 0), 0.8, DriveTimePredictor.Axis.FORWARD, 2).timeS();
        assertTrue("weaker battery, slower move", weak > t2);
    }

    @Test
    public void odometryDriftsOnlyWithErrors() {
        OdometryErrorModel.Params perfect = new OdometryErrorModel.Params(0, 0, 0, 0, 0, Double.NaN, 0);
        OdometryErrorModel.Params noisy = new OdometryErrorModel.Params(0.01, 0.01, 0.05, 0.05, 0.005, Double.NaN, 0);
        OdometryErrorModel a = new OdometryErrorModel(perfect, new Rng(3), 0, 0, 0);
        OdometryErrorModel b = new OdometryErrorModel(noisy, new Rng(3), 0, 0, 0);
        double x = 0;
        for (int i = 0; i < 1000; i++) {
            a.step(0.01, 0, 0.001, 0.01, x, 0, 0);
            b.step(0.01, 0, 0.001, 0.01, x, 0, 0);
            x += 0.01;
        }
        // perfect odometry integrates the same arc exactly
        assertEquals(Math.hypot(a.estX(), a.estY()), Math.hypot(b.estX(), b.estY()), 0.5);
        assertEquals(1.0, a.estHeading(), 1e-9);
        assertTrue(Math.abs(b.estHeading() - 1.0) > 1e-6);
    }
}
