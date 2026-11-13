package org.athletech.sim.scenarios;

import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Tuners.TurretAutotuneEngine;
import org.junit.Test;

import java.util.Locale;
import java.util.Random;

import static org.junit.Assert.assertTrue;

//the turret autotuner against sim turrets it doesn't know, from bad gains and a wrong model guess
public class TurretAutotuneTest {

    static final double H = 0.001;

    static double tpd() {
        return ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    static TurretAutotuneEngine run(CrServoTurret turret, double[] start, long seed) {

        TurretAutotuneEngine engine = new TurretAutotuneEngine(tpd(), start, 8000, 1.0, 1.2);
        Random loops = new Random(seed);
        double t = 0, next = 0;
        while (t < 3000 && !engine.isDone() && !engine.isFaulted()) {
            if (t >= next) {
                engine.update(t, turret.encoderPosition());
                double v = t < TurretPlants.SERVO_WAKE ? 0 : engine.getVolts();
                turret.command(t, v, v);
                next += Math.max(0.012, 0.0255 + 0.0069 * loops.nextGaussian());
            }
            turret.step(t, H);
            t += H;
        }
        System.out.printf(Locale.ROOT, "TURRET autotune: %s after %.0f s%n", engine.getStatus(), t);
        for (String h : engine.getHistory()) System.out.println("    " + h);
        return engine;
    }

    static CrServoTurret turret(double gain, double kinetic, double breakaway, double dead, double startDelay, long seed) {

        CrServoTurret.Params p = new CrServoTurret.Params(gain, 0.046, dead, kinetic, breakaway, 0.44 * tpd(), 26233, 118456);
        p.startDelay = startDelay;
        p.coastDecel = 71702;
        p.neutralDecel = 71702;
        return new CrServoTurret(p, seed);
    }

    static final double[] BAD = {1.5, 1.5, 0, 0, 0, 0, 0, 0, 200, 0.05};

    @Test
    public void measuresTheTurretAndImproves() {

        double[][] plants = {{11374.8, 2.02, 2.33, 0.036, 0.083}, {9566, 1.83, 1.93, 0.066, 0.11}};
        for (int k = 0; k < plants.length; k++) {
            double[] q = plants[k];
            TurretAutotuneEngine e = run(turret(q[0], q[1], q[2], q[3], q[4], 7 + k), BAD, 3 + k);
            assertTrue("finished: " + e.getStatus(), e.isDone());
            System.out.printf(Locale.ROOT, "TURRET autotune plant %d: gain %.0f (true %.0f), friction %.2f / %.2f V (true %.2f / %.2f)%n", k, e.getGain(), q[0], e.getKinetic(), e.getBreakaway(), q[1], q[2]);
            assertTrue("gain within 20%", Math.abs(e.getGain() / q[0] - 1) < 0.2);
            assertTrue("kinetic within 0.3 V", Math.abs(e.getKinetic() - q[1]) < 0.3);
            assertTrue("breakaway within 0.3 V", Math.abs(e.getBreakaway() - q[2]) < 0.3);
            assertTrue("better than it started", e.getResults().get("score_after") < 0.8 * e.getResults().get("score_before"));
            System.out.printf(Locale.ROOT, "TURRET match course plant %d: %.3f -> %.3f%n", k, e.getResults().get("match_score_before"), e.getResults().get("match_score_after"));
            assertTrue("match course no worse", e.getResults().get("match_score_after") <= e.getResults().get("match_score_before"));
        }
    }

    @Test
    public void fromTheCurrentGains() {

        double[] now = {TurretConstants.TURRET_KP_FAR, TurretConstants.TURRET_KP_CLOSE, TurretConstants.TURRET_KI_FAR, TurretConstants.TURRET_KI_CLOSE, TurretConstants.TURRET_KD_FAR, TurretConstants.TURRET_KD_CLOSE, TurretConstants.TURRET_K_HOLD_VOLTS, TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES, TurretConstants.TURRET_MAX_I, TurretConstants.TURRET_STRIBECK_DEGREES_PER_SECOND};
        TurretAutotuneEngine e = run(turret(11374.8, 2.02, 2.33, 0.036, 0.083, 9), now, 9);
        assertTrue("finished: " + e.getStatus(), e.isDone());
        assertTrue("no worse than it started", e.getResults().get("score_after") <= e.getResults().get("score_before"));
    }

    @Test
    public void catchesADeadEncoder() {

        CrServoTurret t = turret(11374.8, 2.02, 2.33, 0.036, 0.083, 1);
        t.encoderSign = 0;
        TurretAutotuneEngine e = run(t, BAD, 1);
        assertTrue("faulted", e.isFaulted());
        assertTrue("turret stopped near home", Math.abs(t.position()) / tpd() < 120);
    }

    //the robot as measured, servos asleep for the first 1.1 s: a plugged-in encoder must pass
    @Test
    public void pluggedInEncoderPasses() {

        for (long seed = 1; seed <= 4; seed++) {
            for (double sticky : new double[] {1, 1.15}) {
                TurretAutotuneEngine e = run(new CrServoTurret(TurretPlants.params(1, 1, sticky), seed), BAD, seed);
                boolean encoderFault = e.isFaulted() && e.getStatus().contains("encoder");
                assertTrue("seed " + seed + " sticky " + sticky + ": " + e.getStatus(), !encoderFault);
                assertTrue("both kicks passed", e.getHistory().toString().split("encoder ok", -1).length - 1 == 2);
            }
        }
    }

    //B during the match course keeps the step-move result
    @Test
    public void keepsWhatsTunedWhenStoppedOnTheCourse() {

        CrServoTurret turret = new CrServoTurret(TurretPlants.params(1, 1), 3);
        TurretAutotuneEngine engine = new TurretAutotuneEngine(tpd(), BAD, 8000, 1.0, 1.2);
        double t = 0, next = 0;
        boolean kept = false;
        while (t < 3000 && !engine.isDone() && !engine.isFaulted()) {
            if (t >= next) {
                engine.update(t, turret.encoderPosition());
                if (!kept && "MATCH".equals(engine.getStage())) kept = engine.keepEarly();
                double v = t < TurretPlants.SERVO_WAKE ? 0 : engine.getVolts();
                turret.command(t, v, v);
                next += 0.025;
            }
            turret.step(t, H);
            t += H;
        }
        assertTrue("kept", kept && engine.isDone());
        assertTrue("has the gains", engine.getResults().containsKey("kp_far") && engine.getResults().containsKey("pulse_zone_degrees"));
    }
}
