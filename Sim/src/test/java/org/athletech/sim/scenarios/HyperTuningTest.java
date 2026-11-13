package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.Findings;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.FlywheelHyperTuner;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.OnlineTuner;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.TurretHyperTuner;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class HyperTuningTest {

    @Test
    public void onlineTunerFindsTheBottomThroughNoise() {

        Random rnd = new Random(1);
        OnlineTuner t = new OnlineTuner("bowl", 3, new OnlineTuner.Knob("a", 0, -5, 5, 1, 0.05), new OnlineTuner.Knob("b", 0, -5, 5, 1, 0.05));
        for (int k = 0; k < 600; k++) {
            double a = t.value("a"), b = t.value("b");
            double cost = 1 + (a - 3) * (a - 3) + 2 * (b + 1) * (b + 1);
            t.submit(cost * (1 + 0.05 * rnd.nextGaussian()));
            assertTrue(a >= -5 && a <= 5 && b >= -5 && b <= 5);
        }
        assertEquals(3, t.value("a"), 0.6);
        assertEquals(-1, t.value("b"), 0.5);
        assertTrue(t.baseline < 0.5 * t.firstBaseline);
    }

    @Test
    public void onlineTunerDoesNotChaseNoise() {

        Random rnd = new Random(2);
        OnlineTuner t = new OnlineTuner("flat", 3, new OnlineTuner.Knob("a", 1, 0, 2, 0.2, 0.01));
        for (int k = 0; k < 300; k++) t.submit(10 * (1 + 0.01 * rnd.nextGaussian()));
        assertEquals(1, t.value("a"), 0.25);
    }

    @Test
    public void flywheelTunerScoresBurstsAndSteadyWindows() {

        FlywheelPIDVSCoefficients c = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        FlywheelPIDVSCoefficients copy = new FlywheelPIDVSCoefficients(c.kpFar, c.kpClose, c.kiFar, c.kiClose, c.kd, c.kv, c.ks, c.ksSignWidth, c.iSwitch, c.pSwitch, c.kISmash, c.minP, c.maxP, c.minI, c.maxI, c.minD, c.maxD);
        double small = burstCost(copy, 40), big = burstCost(copy, 120);
        assertTrue("small dip " + small + " big dip " + big, big > small && small > 0);
    }

    static double burstCost(FlywheelPIDVSCoefficients c, double dip) {

        FlywheelHyperTuner h = new FlywheelHyperTuner(c, 0, 0, 0.08, 0, 0.04);
        Random rnd = new Random(4);
        double v = 1450, t = 0;
        int shots = 0;
        for (int k = 0; k < 2000; k++, t += 0.02) {
            boolean feeding = t > 10 && t < 11.2;
            if (feeding && k % 15 == 0) {
                v -= dip;
                shots++;
            }
            v += (1450 - v) * 0.15;
            h.update(t, 1450, v + 2 * rnd.nextGaussian(), feeding, shots);
        }
        assertTrue("steady windows scored", !Double.isNaN(h.lastSteadyCost));
        assertEquals(1, h.bursts);
        return h.lastShotCost;
    }

    @Test
    public void findingsCallOutAPinnedKnob() {

        FlywheelPIDVSCoefficients c = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        FlywheelHyperTuner f = new FlywheelHyperTuner(c, 0, 0, 0.08, 0, 0.04);
        OnlineTuner.Knob boost = f.shots.knob("feed_boost_volts");
        boost.value = boost.max;
        boost.kept = 3;
        boost.atBound = 2;
        String md = Findings.markdown("test", 60, f, new TurretHyperTuner());
        assertTrue(md, md.contains("feed_boost_volts wants past its top"));
        assertTrue(md, md.contains("iterative learning control"));
    }

    //a shaky hold costs more than a still one, shaking while a shot is fed costs more again, and moving aims are scored apart
    @Test
    public void turretTunerScoresHoldAndTrack() {

        double still = holdCost(0, false), shaky = holdCost(3, false), shakyFeeding = holdCost(3, true);
        assertTrue("still " + still + " shaky " + shaky, shaky > 2 * still);
        assertTrue("feeding counts more: " + shakyFeeding + " vs " + shaky, shakyFeeding > shaky);
        TurretHyperTuner h = new TurretHyperTuner();
        for (int k = 0; k < 200; k++) h.update(k * 0.02, 0.02, 20 * k * 0.02, 20 * k * 0.02 - 1.5, 20, 1, false);
        assertEquals(1.5, h.lastTrackCost, 1e-6);
    }

    static double holdCost(double shake, boolean feeding) {

        TurretHyperTuner h = new TurretHyperTuner();
        for (int k = 0; k < 300; k++) {
            double t = k * 0.02, pos = 0.4 + shake * Math.sin(2 * Math.PI * 4 * t);
            h.update(t, 0.02, 0, pos, 0, Math.cos(2 * Math.PI * 4 * t), feeding && t > 1);
        }
        return h.lastHoldCost;
    }
}
