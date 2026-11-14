package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.Findings;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.FlywheelHyperTuner;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.OnlineTuner;
import org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning.TurretHyperTuner;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

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

        double small = burstCost(50), big = burstCost(120);
        assertTrue("small dip " + small + " big dip " + big, big > small && small > 0);
    }

    static FlywheelHyperTuner tuner() {

        FlywheelPIDVSCoefficients c = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        FlywheelPIDVSCoefficients copy = new FlywheelPIDVSCoefficients(c.kpFar, c.kpClose, c.kiFar, c.kiClose, c.kd, c.kv, c.ks, c.ksSignWidth, c.iSwitch, c.pSwitch, c.kISmash, c.minP, c.maxP, c.minI, c.maxI, c.minD, c.maxD);
        return new FlywheelHyperTuner(copy, 0, 0, 0.08, 0, 0.04);
    }

    static double burstCost(double dip) {

        FlywheelHyperTuner h = run(new double[] {10.2, 10.5, 10.8, 11.1}, new double[] {dip, dip, dip, dip}, 10, 11.2, 0, 3);
        assertTrue("steady windows scored", !Double.isNaN(h.lastSteadyCost));
        assertEquals(1, h.bursts);
        assertEquals(4, h.balls);
        return h.lastShotCost;
    }

    //balls hit at ballTimes and pull the wheel down over `spread` loops; after the feed the wheel can overshoot by `overshoot`
    static FlywheelHyperTuner run(double[] ballTimes, double[] dips, double feedFrom, double feedTo, double overshoot, int spread) {

        FlywheelHyperTuner h = tuner();
        Random rnd = new Random(4);
        double v = 1450, t = 0, pull = 0;
        int left = 0;
        for (int k = 0; k < 1000; k++, t += 0.02) {
            boolean feeding = t > feedFrom && t < feedTo;
            for (int b = 0; b < ballTimes.length; b++) {
                if (Math.abs(t - ballTimes[b]) < 0.01) {
                    pull = dips[b] / spread;
                    left = spread;
                }
            }
            if (left > 0) {
                v -= pull;
                left--;
            }
            else {
                double aim = 1450 + (t > feedTo + 0.4 && t < feedTo + 0.7 ? overshoot : 0);
                v += (aim - v) * 0.15;
            }
            h.update(t, 1450, v + 2 * rnd.nextGaussian(), feeding);
        }
        return h;
    }

    @Test
    public void flywheelTunerCountsEachBallOnceEvenLateOrSlow() {

        //a slow dip over 4 loops is one ball, and one landing 0.5 s after the blocker closed still counts
        FlywheelHyperTuner h = run(new double[] {10.2, 11.7}, new double[] {90, 90}, 10, 11.2, 0, 4);
        assertEquals(2, h.balls);
        assertEquals(1, h.bursts);
    }

    @Test
    public void flywheelTunerIgnoresOvershootAndScoresIt() {

        //settling down from an overshoot is not a ball, but the overshoot just after the burst costs
        FlywheelHyperTuner calm = run(new double[] {10.2}, new double[] {90}, 10, 10.4, 0, 2);
        FlywheelHyperTuner over = run(new double[] {10.2}, new double[] {90}, 10, 10.4, 60, 2);
        assertEquals(1, over.balls);
        assertTrue("overshoot " + over.lastShotCost + " vs calm " + calm.lastShotCost, over.lastShotCost > calm.lastShotCost + 10);
        assertTrue("overshoot " + over.lastOvershoot, over.lastOvershoot > 30);
    }

    @Test
    public void flywheelTunerWeighsLaterBallsMore() {

        //the same slow-to-recover ball costs more as the third of a burst than as the first
        FlywheelHyperTuner first = run(new double[] {10.2, 10.6, 11.0}, new double[] {160, 60, 60}, 10, 11.2, 0, 2);
        FlywheelHyperTuner third = run(new double[] {10.2, 10.6, 11.0}, new double[] {60, 60, 160}, 10, 11.2, 0, 2);
        assertEquals(3, first.balls);
        assertEquals(3, third.balls);
        assertTrue("third " + third.lastShotCost + " vs first " + first.lastShotCost, third.lastShotCost > first.lastShotCost);
    }

    //a knob that does nothing: noise alone used to get ~37% of trials kept at 15% noise (each keep also took the lucky low cost as the
    //new baseline); now it measures longer when noisy and keeps under 15%
    @Test
    public void onlineTunerSamplesMoreWhenNoisyAndRarelyKeepsNoise() {

        int[] clean = flat(0.002), noisy = flat(0.15);
        double cleanSamples = (double) clean[0] / clean[1], noisySamples = (double) noisy[0] / noisy[1];
        assertTrue("samples clean " + cleanSamples + " noisy " + noisySamples, noisySamples > cleanSamples + 1);
        assertTrue("kept " + noisy[2] + " of " + noisy[1], noisy[2] < 0.15 * noisy[1]);
    }

    //{submits, trials, keeps} on a cost no knob changes
    static int[] flat(double noise) {

        Random rnd = new Random(5);
        OnlineTuner t = new OnlineTuner("flat", 2, new OnlineTuner.Knob("a", 1, 0, 2, 0.2, 0.01));
        int submits = 0;
        while (t.trials < 400) {
            t.submit(10 * (1 + noise * rnd.nextGaussian()));
            submits++;
        }
        return new int[] {submits, t.trials, t.keeps};
    }

    //the real 2028-05-13 TeleOp, when its log is pulled: the tuner finds the balls tools/shot_video.py finds (64)
    @Test
    public void flywheelTunerFindsTheBallsInARealMatchLog() throws Exception {

        File run = new File("../tuning-data/robot-logs/20280513-230945-RedTeleOp");
        Assume.assumeTrue(new File(run, "frames.bin").isFile());
        List<String> names = new ArrayList<>();
        List<String> lines = Files.readAllLines(new File(run, "channels.tsv").toPath(), StandardCharsets.UTF_8);
        for (String line : lines.subList(1, lines.size())) {
            String[] p = line.split("	");
            names.add(p[1] + "." + p[2]);
        }
        int n = names.size(), time = names.indexOf("time.t"), target = names.indexOf("flywheel.target"), velocity = names.indexOf("flywheel.velocity"), clear = names.indexOf("blocker.clear");
        ByteBuffer data = ByteBuffer.wrap(Files.readAllBytes(new File(run, "frames.bin").toPath())).order(ByteOrder.LITTLE_ENDIAN);
        FlywheelHyperTuner h = tuner();
        for (int row = 0; (row + 1) * n * 8 <= data.capacity(); row++) {
            int at = row * n * 8;
            h.update(data.getDouble(at + 8 * time), data.getDouble(at + 8 * target), data.getDouble(at + 8 * velocity), data.getDouble(at + 8 * clear) == 1);
        }
        assertTrue("balls " + h.balls + " bursts " + h.bursts, Math.abs(h.balls - 64) <= 6 && h.bursts > 25);
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
