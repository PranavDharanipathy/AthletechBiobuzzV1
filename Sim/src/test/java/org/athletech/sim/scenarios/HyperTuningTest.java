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
    public void spinShareConvergesToMeetingTheSpin() {

        for (double start : new double[] {0.75, 1.25}) {
            TurretHyperTuner h = new TurretHyperTuner();
            h.rateScale = start;
            Random rnd = new Random(3);
            double t = 0;
            for (int k = 0; k < 3000; k++, t += 0.02) {
                double rate = 180 * Math.signum(Math.sin(0.5 * t) + 1e-9);
                double target = 0, lag = (1 - h.rateScale) * rate * 0.1;
                h.update(t, target, target - lag + 0.2 * rnd.nextGaussian(), rate, 2, rate, rate, true);
            }
            assertEquals("from " + start, 1, h.rateScale, 0.03);
        }
    }

    @Test
    public void flywheelTunerScoresBurstsAndSteadyWindows() {

        FlywheelPIDVSCoefficients c = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        FlywheelPIDVSCoefficients copy = new FlywheelPIDVSCoefficients(c.kpFar, c.kpClose, c.kiFar, c.kiClose, c.kd, c.kv, c.ks, c.ksSignWidth, c.iSwitch, c.pSwitch, c.kISmash, c.minP, c.maxP, c.minI, c.maxI, c.minD, c.maxD);
        double small = burstCost(copy, 40), big = burstCost(copy, 120);
        assertTrue("small dip " + small + " big dip " + big, big > small && small > 0);
    }

    static double burstCost(FlywheelPIDVSCoefficients c, double dip) {

        FlywheelHyperTuner h = new FlywheelHyperTuner(c, 0, 0, 0.08);
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
        FlywheelHyperTuner f = new FlywheelHyperTuner(c, 0, 0, 0.08);
        OnlineTuner.Knob boost = f.shots.knob("feed_boost_volts");
        boost.value = boost.max;
        boost.kept = 3;
        boost.atBound = 2;
        String md = Findings.markdown("test", 60, f, new TurretHyperTuner());
        assertTrue(md, md.contains("feed_boost_volts wants past its top"));
        assertTrue(md, md.contains("iterative learning control"));
    }
}
