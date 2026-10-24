package org.athletech.sim;

import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.DcMotor;
import org.athletech.sim.predict.LoopControlSim;
import org.athletech.sim.robot.MatchDriver;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.scenarios.LoopTimingScenario;
import org.athletech.sim.undeterminables.ChargeSim;
import org.athletech.sim.undeterminables.Disconnects;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.undeterminables.Electrostatics;
import org.athletech.sim.undeterminables.LoopTimingModel;
import org.athletech.sim.undeterminables.RobotCharge;
import org.athletech.sim.undeterminables.TileChargeGrid;
import org.athletech.sim.utility.Rng;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Loop timing, static charge and disconnect models. */
public class UndeterminablesTest {

    static LoopTimingModel.Params parts(double jitterStd, double spikeProb) {
        return new LoopTimingModel.Params(3, 0.6, 4, 1.2, 4, 1.0, 2, 2.5, 1, 0.5, 1, 0.3,
                Distribution.normal("jitter", 0, jitterStd, "ms"), spikeProb, Distribution.normal("spike", 20, 5, "ms"));
    }

    @Test
    public void loopTimeIsTheSumOfItsParts() {
        LoopTimingModel.Params p = parts(0.5, 0);
        assertEquals(3 + 0.6 * 4 + 1.2 * 4 + 1.0 * 2 + 2.5 + 0.5, p.meanMs(), 1e-9);
        LoopTimingModel m = LoopTimingModel.of(p);
        Rng rng = new Rng(3);
        double sum = 0;
        int n = 20000;
        for (int i = 0; i < n; i++) {
            LoopTimingModel.Loop l = m.next(rng);
            assertTrue("sense before act before end", l.senseMs <= l.actMs && l.actMs <= l.periodMs);
            sum += l.periodMs;
        }
        assertEquals(p.meanMs(), sum / n, 0.02 * p.meanMs());
        // one more I2C read costs its read time
        assertEquals(p.meanMs() + 2.5, p.withSensorReads(2).meanMs(), 1e-9);
        // spikes raise the average by probability x spike length
        assertEquals(p.meanMs() + 0.01 * 20, parts(0.5, 0.01).meanMs(), 1e-9);
    }

    @Test
    public void slowLoopsTurnWorse() {
        MecanumDrive.Params dp = PhysicsTest.drive(0.9, 0.04, 0.8);
        Supplier<Battery> bat = () -> new Battery(13, 0.07, 0);
        double tol = Math.toRadians(1);
        LoopControlSim.Gains g = LoopControlSim.tune(dp, bat, 0.8, true, Math.PI / 2, tol, 200, new Rng(1));
        LoopControlSim.StepResult fast = LoopControlSim.step(dp, bat, 0.8, LoopTimingModel.synthetic(3, 0.5, 0.3, 0.3),
                g.kp, g.kd, 200, true, Math.PI / 2, tol, 2.5, new Rng(1));
        LoopControlSim.StepResult slow = LoopControlSim.step(dp, bat, 0.8, LoopTimingModel.synthetic(45, 8, 0.3, 0.3),
                g.kp, g.kd, 200, true, Math.PI / 2, tol, 2.5, new Rng(1));
        assertTrue("fast loop settles", Double.isFinite(fast.settleS));
        assertTrue("slow loop overshoots more: " + fast.overshoot + " vs " + slow.overshoot, slow.overshoot > fast.overshoot);
        double slowSettle = Double.isFinite(slow.settleS) ? slow.settleS : 2.5;
        assertTrue("slow loop settles later: " + fast.settleS + " vs " + slowSettle, slowSettle > fast.settleS);
    }

    @Test
    public void arcIntegrationBeatsStraightSteps() {
        LoopTimingModel m = LoopTimingModel.synthetic(30, 5, 0.3, 0.3);
        double straight = LoopControlSim.odometryErrorPerM(m, false, 8, new Rng(2));
        double arc = LoopControlSim.odometryErrorPerM(m, true, 8, new Rng(2));
        assertTrue("arc " + arc + " < straight " + straight, arc < straight);
        double fine = LoopControlSim.odometryErrorPerM(LoopTimingModel.synthetic(2, 0, 0.3, 0.3), false, 8, new Rng(2));
        assertTrue("short loops integrate better: " + fine + " vs " + straight, fine < straight);
    }

    @Test
    public void internalArcReleasesTheCapacitorEnergy() {
        double cf = 60e-12, ce = 30e-12, cfe = 300e-12;
        RobotCharge r = new RobotCharge(cf, ce, cfe);
        r.addFrame(500e-9);
        double dv = r.vElec(0) - r.vFrame(0);
        double ceq = cfe + cf * ce / (cf + ce);
        double e = r.equalizeInternal(0);
        assertEquals(0.5 * ceq * dv * dv, e, 1e-12 * Math.abs(e) + 1e-15);
        assertEquals(0, r.vElec(0) - r.vFrame(0), 1e-6);
        assertEquals(500e-9, r.qFrame() + r.qElec(), 1e-18); // charge is conserved
        assertEquals(0.5 * 1e-12 * 2e-12 / 3e-12 * 100 * 100, Electrostatics.contactEnergy(1e-12, 2e-12, 100), 1e-20);
    }

    @Test
    public void strapBleedsTheElectronicsToFrameDifference() {
        RobotCharge strap = new RobotCharge(60e-12, 30e-12, 300e-12), none = new RobotCharge(60e-12, 30e-12, 300e-12);
        strap.addFrame(500e-9);
        none.addFrame(500e-9);
        for (int i = 0; i < 10; i++) {
            strap.step(0.001, 0, 0, 1.0 / 470);   // REV strap: 470 ohm
            none.step(0.001, 0, 0, 1e-12);   // insulation only
        }
        assertTrue(Math.abs(strap.vElec(0) - strap.vFrame(0)) < 1);
        assertTrue(Math.abs(none.vElec(0) - none.vFrame(0)) > 100);
        // leaking to the floor: tau = R C; after one tau about 37% is left
        RobotCharge leak = new RobotCharge(100e-12, 30e-12, 300e-12);
        leak.addFrame(1e-6);
        double v0 = leak.vFrame(0);
        double g = 1e-10; // 10 GOhm
        double cTotal = 1 / leak.frameElastance(); // what the frame looks like from the floor (electronics floating)
        double tau = cTotal / g;
        int steps = 1000;
        for (int i = 0; i < steps; i++) leak.step(tau / steps, 0, g, 1e-12);
        assertEquals(Math.exp(-1), leak.vFrame(0) / v0, 0.01);
    }

    @Test
    public void frameCapacitanceIsPlausible() {
        double c = Electrostatics.robotCapacitance(0.43, 0.43, 0.03, 0.5, 0.016, 1.4) * 1e12;
        assertTrue("pF " + c, c > 20 && c < 200);
        double f = Electrostatics.inducedFraction(0.03, 0.016, 1.4);
        assertTrue(f > 0 && f < 1);
        assertTrue(Electrostatics.resistanceFactor(20, 1) > Electrostatics.resistanceFactor(50, 1));
        assertTrue(Electrostatics.generationFactor(20, 15) > Electrostatics.generationFactor(50, 15));
    }

    @Test
    public void humidityDrainsTheCharge() {
        MecanumDrive.Params dp = PhysicsTest.drive(0.9, 0.04, 0.8);
        MatchDriver.Behaviour b = new MatchDriver.Behaviour();
        b.autoS = 10;
        b.transitionS = 2;
        b.teleopS = 30;
        MatchDriver.Trace tr = MatchDriver.simulate(dp, new Battery(13, 0.07, 0.6), 0.8, null, b, new Rng(4));
        assertTrue("drove somewhere: " + tr.distanceM, tr.distanceM > 5);
        ChargeSim.Params p = new ChargeSim.Params();
        p.autoS = b.autoS;
        p.transitionS = b.transitionS;
        p.teleopS = b.teleopS;
        double dry = ChargeSim.runMatch(p.withRh(15), tr, new ChargeSim.Field(p.fieldSize), new Rng(5), false).maxAbsKv;
        double humid = ChargeSim.runMatch(p.withRh(60), tr, new ChargeSim.Field(p.fieldSize), new Rng(5), false).maxAbsKv;
        assertTrue("dry " + dry + " kV > humid " + humid + " kV", dry > 3 * humid);
        assertTrue("corona caps it: " + dry, dry < p.coronaV / 1000 * 1.3);
    }

    @Test
    public void brownoutsAndFragility() {
        float[] v = new float[100];
        Arrays.fill(v, 12f);
        for (int i = 10; i < 13; i++) v[i] = 6.5f;   // 3 ms dip
        v[50] = 6.5f;                                 // 1 ms blip
        List<Double> b = Disconnects.brownouts(v, 0.001, 7, 0.002);
        assertEquals(1, b.size());
        assertEquals(0.011, b.get(0), 1e-9);
        assertEquals(0.5, Disconnects.fragility(10, 10, 0.6), 1e-6);
        assertTrue(Disconnects.fragility(20, 10, 0.6) > Disconnects.fragility(15, 10, 0.6));
        assertEquals(0, Disconnects.fragility(5, Double.NaN, 0.6), 0);

        Disconnects.Mode hub = new Disconnects.Mode("control_hub_reboot", true, Double.NaN, 7, Double.NaN, Double.NaN, Double.NaN, 0, 0,
                Distribution.normal("out", 20, 0, "s"));
        float[] cur = new float[100];
        Disconnects.Stress s = new Disconnects.Stress(0.1, 0.1, new ArrayList<double[]>(), new ArrayList<double[]>(), v, cur, 0.001, 0.002, 0.004);
        Disconnects.Result r = Disconnects.evaluate(Collections.singletonList(hub), s, 0.6, new Rng(1));
        assertEquals(1, r.atLeastOne[0], 0);
        Arrays.fill(v, 12f);
        r = Disconnects.evaluate(Collections.singletonList(hub), s, 0.6, new Rng(1));
        assertEquals(0, r.atLeastOne[0], 0);
        assertEquals(0, r.downtimeS, 0);
    }

    @Test
    public void slippingWheelsSpinUpUntilTractionHoldsAndDrawLessCurrent() {
        double mu = 0.5;
        MecanumDrive d = new MecanumDrive(PhysicsTest.drive(mu, 0, 0.8));
        d.reset(0, 0, 0);
        d.drive(1, 0, 0);
        d.step(0.001, new Battery(12, 0, 0), 0);
        assertTrue(d.anySlipping());
        double cap = mu * 13 * 9.80665 / 4 / Math.sqrt(2), radius = 0.052;
        // the slipping wheel turns at the speed where the motor's torque equals what traction holds
        double omega = d.wheelSlip(MecanumDrive.FL) / radius; // the chassis was at rest
        DcMotor m = new DcMotor(PhysicsTest.YJ312);
        m.setDuty(1);
        assertEquals(cap * radius, m.torque(m.current(12, omega), omega), 1e-6);
        // so each motor draws that speed's current, well under stall current
        assertEquals(4 * m.current(12, omega), d.motorCurrentTotal(), 1e-6);
        assertTrue(d.motorCurrentTotal() < 0.9 * 4 * PhysicsTest.YJ312.stallCurrentA());
        // plenty of traction: no slip
        MecanumDrive g = new MecanumDrive(PhysicsTest.drive(5, 0, 0.8));
        g.reset(0, 0, 0);
        g.drive(1, 0, 0);
        g.step(0.001, new Battery(12, 0, 0), 0);
        assertTrue(!g.anySlipping());
        assertEquals(0, g.slipSpeedMean(), 0);
    }

    @Test
    public void everyCoulombIsAccountedFor() {
        MecanumDrive.Params dp = PhysicsTest.drive(0.9, 0.04, 0.8);
        MatchDriver.Behaviour b = new MatchDriver.Behaviour();
        b.autoS = 10;
        b.transitionS = 2;
        b.teleopS = 40;
        b.wallHitsPerMatch = 8;
        b.robotContactsPerMatch = 6;
        b.metalContactsPerMatch = 4;
        MatchDriver.Trace tr = MatchDriver.simulate(dp, new Battery(13, 0.07, 0.6), 0.8, null, b, new Rng(9));
        assertTrue("pushing walls makes the wheels slip: " + tr.slipM + " m", tr.slipM > 0);
        ChargeSim.Params p = new ChargeSim.Params();
        p.autoS = b.autoS;
        p.transitionS = b.transitionS;
        p.teleopS = b.teleopS;
        p.cMetal = 100e-12;
        p.tileRhoS40 = 1e12; // sideways creep on
        p.wallHitsPerMatch = b.wallHitsPerMatch;
        p.robotContactsPerMatch = b.robotContactsPerMatch;
        p.metalContactsPerMatch = b.metalContactsPerMatch;
        for (double rh : new double[]{15, 45}) {
            ChargeSim.Field field = new ChargeSim.Field(p.fieldSize);
            ChargeSim.MatchResult r = ChargeSim.runMatch(p.withRh(rh), tr, field, new Rng(10), false);
            assertTrue(r.globalTurnoverC > 0);
            assertEquals("robot ledger at " + rh + "%", 0, r.ledger.residual(), 1e-9 * r.globalTurnoverC);
            assertEquals("whole field at " + rh + "%", 0, r.globalResidualC, 1e-9 * r.globalTurnoverC);
            assertTrue("slip charged the robot", r.ledger.c[ChargeSim.Ledger.SLIP] > 0);
            assertTrue("the floor drained some", r.ledger.c[ChargeSim.Ledger.FLOOR] < 0);
        }
    }

    @Test
    public void tileChargeCreepsSidewaysWithoutBeingLost() {
        TileChargeGrid g = new TileChargeGrid(3.6576, 24);
        g.deposit(0.1, 0.1, 1e-6);
        g.deposit(5, 5, -2e-7); // outside the walls: lands in the corner cell, not lost
        assertEquals(8e-7, g.total(), 1e-18);
        g.diffuse(1e-3, 60, 100000);
        assertEquals(8e-7, g.total(), 1e-18);
        assertTrue("spread out", g.maxAbsCell() < 0.5e-6);
        g.diffuse(10, 1e4, 100); // very conductive: evens out
        assertEquals(8e-7 / (24 * 24), g.maxAbsCell(), 1e-15);
    }

    @Test
    public void shortPulsesSlipBetweenReads() {
        double[] steady = new double[1000];
        Arrays.fill(steady, 10);
        assertEquals(0.6, LoopTimingScenario.missedEventProb(steady, 4), 1e-12);
        assertEquals(0, LoopTimingScenario.missedEventProb(steady, 10), 1e-12);
        assertEquals(5, LoopTimingScenario.timerOvershootMs(steady), 1e-12);
        // uneven loops: the formula matches a direct simulation of reads and random pulses
        LoopTimingModel m = LoopTimingModel.synthetic(12, 6, 0.3, 0.3);
        Rng rng = new Rng(4);
        double[] periods = m.samplePeriods(rng, 200000);
        double w = 5;
        double[] reads = new double[periods.length + 1];
        for (int i = 0; i < periods.length; i++) reads[i + 1] = reads[i] + periods[i];
        int missed = 0, n = 200000;
        Rng r2 = new Rng(5);
        for (int k = 0; k < n; k++) {
            double s = r2.uniform(0, reads[reads.length - 1] - 100);
            int i = Arrays.binarySearch(reads, s);
            int next = i >= 0 ? i : -i - 1; // first read at or after the pulse start
            if (reads[next] > s + w) missed++;
        }
        assertEquals(LoopTimingScenario.missedEventProb(periods, w), (double) missed / n, 0.006);
        assertTrue(LoopTimingScenario.timerOvershootMs(periods) > 6); // longer than T/2: long loops are hit more often
        double pulse = LoopTimingScenario.minReliablePulseMs(periods, 0.01);
        assertEquals(0.01, LoopTimingScenario.missedEventProb(periods, pulse), 1e-4);
    }
}
