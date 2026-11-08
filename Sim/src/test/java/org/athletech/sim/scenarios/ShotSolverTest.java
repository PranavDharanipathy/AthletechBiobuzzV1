package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelVelocityModel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotSolver;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

//shoot on the move with the flywheel in it, checked against real ball flight: a launcher whose exit speed matches the tuned
//table when standing still (that's how it was tuned), the robot's velocity added to the ball, no drag. Where does the ball
//cross the CELL's height on the way down, against where the goal is?
public class ShotSolverTest {

    //the 2026-11 tuned table, fixed here so the test doesn't move when ShooterConstants is retuned
    static final java.util.List<Double> D = java.util.Arrays.asList(51.71503320693509, 56.29514009214838, 60.899456934836486, 68.99167060900542, 74.57368259902958, 80.60268703640344, 91.2495467341891, 100.4429764662134);
    static final java.util.List<Double> V = java.util.Arrays.asList(1300.0, 1330.0, 1380.0, 1440.0, 1455.0, 1485.0, 1590.0, 1665.0);

    private FlywheelVelocityModel model;
    private ShotSolver solver;
    private static final double EXTRAPOLATE = 12;
    private boolean sotmBefore;

    //these check the shoot-on-the-move math itself, whatever the robot's switch is set to
    @Before
    public void setUp() {

        sotmBefore = ShooterConstants.SOTM_ENABLED;
        ShooterConstants.SOTM_ENABLED = true;
        model = new FlywheelVelocityModel();
        model.velocity(D, V, 1, EXTRAPOLATE, 75);
        solver = new ShotSolver(model);
    }

    @After
    public void restore() {
        ShooterConstants.SOTM_ENABLED = sotmBefore;
    }

    //the "real" ball for a flywheel speed: launched like a still shot to the distance that speed was tuned for
    private double[] fly(double flywheel, double aimX, double aimY, double vx, double vy) {

        double d = model.distanceFor(flywheel, EXTRAPOLATE);
        double th = Math.toRadians(ShooterConstants.SHOT_LAUNCH_ANGLE_DEGREES), rise = ShooterConstants.SHOT_RISE_INCHES, g = ShotSolver.GRAVITY;
        double still = d / Math.sqrt(2 * (d * Math.tan(th) - rise) / g);
        double horizontal = still, vertical = still * Math.tan(th);
        double t = (vertical + Math.sqrt(vertical * vertical - 2 * g * rise)) / g;
        double n = Math.hypot(aimX, aimY);
        return new double[] {(horizontal * aimX / n + vx) * t, (horizontal * aimY / n + vy) * t};
    }

    //what the Shooter does: flywheel for the equivalent still distance, turret along goal - v T
    private double landingError(double gx, double gy, double vx, double vy, double flywheel) {

        double miss = solver.missInches(gx, gy, vx, vy, flywheel, EXTRAPOLATE);
        assertTrue("no solution", miss == miss);
        double t = solver.time;
        double[] at = fly(flywheel, gx - vx * t, gy - vy * t, vx, vy);
        return Math.hypot(at[0] - gx, at[1] - gy);
    }

    //every direction and speed up to 60 in/s across the tuned range: the speed it picks plus its aim put the ball in the goal
    @Test
    public void movingShotsLandOnTheGoal() {

        double worst = 0;
        for (double dist = 56; dist <= 96; dist += 10) {
            for (int a = 0; a < 360; a += 30) {
                for (double speed = 0; speed <= 60; speed += 20) {
                    double vx = speed * Math.cos(Math.toRadians(a)), vy = speed * Math.sin(Math.toRadians(a));
                    double d = solver.equivalentDistance(dist, 0, vx, vy);
                    double flywheel = model.velocity(D, V, 1, EXTRAPOLATE, d);
                    if (d < model.minDistance() - EXTRAPOLATE || d > model.maxDistance() + EXTRAPOLATE) continue;
                    worst = Math.max(worst, landingError(dist, 0, vx, vy, flywheel));
                }
            }
        }
        assertTrue("worst landing error " + worst + " in", worst < 0.05);
    }

    //the flywheel has to change with the motion: at the goal slower than standing still, away and across faster
    @Test
    public void flywheelSpeedFollowsTheMotion() {

        double still = solver.equivalentDistance(75, 0, 0, 0);
        assertEquals(75, still, 1e-9);
        double toward = solver.equivalentDistance(75, 0, 40, 0), away = solver.equivalentDistance(75, 0, -40, 0), across = solver.equivalentDistance(75, 0, 0, 40);
        assertTrue(toward + " " + still, toward < still - 20);
        assertTrue(away + " " + still, away > still + 20);
        assertTrue(across + " " + still, across > still + 5);
    }

    //a lagging or dipping flywheel: missInches says how far long/short the ball really lands, so the blocker can hold it
    @Test
    public void missMatchesWhereTheBallReallyLands() {

        double vx = 25, vy = -30, gx = 70, gy = 20;
        double d = solver.equivalentDistance(gx, gy, vx, vy);
        double right = model.velocity(D, V, 1, EXTRAPOLATE, d);
        for (double off = -60; off <= 60; off += 20) {
            double miss = solver.missInches(gx, gy, vx, vy, right + off, EXTRAPOLATE);
            double t = solver.time;
            double ax = gx - vx * t, ay = gy - vy * t, n = Math.hypot(ax, ay);
            double[] at = fly(right + off, ax, ay, vx, vy);
            double alongAim = ((at[0] - gx) * ax + (at[1] - gy) * ay) / n, sideways = ((at[0] - gx) * -ay + (at[1] - gy) * ax) / n;
            assertEquals("off " + off, alongAim, miss, 0.05);
            assertEquals("off " + off, 0, sideways, 0.05);
        }
        assertTrue(Math.abs(solver.missInches(gx, gy, vx, vy, right, EXTRAPOLATE)) < 0.05);
        assertTrue(Math.abs(solver.missInches(gx, gy, vx, vy, right + 40, EXTRAPOLATE)) > ShooterConstants.SHOT_MISS_TOLERANCE_INCHES);
    }

    //standing still it's just the speed fit
    @Test
    public void stillShotIsThePlainFit() {

        double miss = solver.missInches(80, 0, 0, 0, model.velocity(D, V, 1, EXTRAPOLATE, 80), EXTRAPOLATE);
        assertEquals(0, miss, 1e-6);
        assertTrue(Double.isNaN(solver.missInches(80, 0, 0, 0, 5000, EXTRAPOLATE)));
    }
}
