package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.shooter.core.*;
import org.firstinspires.ftc.teamcode.shooter.sim.FlywheelSim;
import org.firstinspires.ftc.teamcode.shooter.sim.TurretSim;

import java.util.Random;

/**
 * Off-robot verification of the shooter core against plant simulators. It is a plain Java main(), with no
 * test-framework dependency (team rule: no third-party libraries). Run with {@code tools/run_tests.sh}.
 * Each check prints a measured value next to its requirement (see docs/SHOOTER_DESIGN.md §2).
 */
public final class ShooterTests {

    private static int failures = 0, checks = 0;

    private static void check(String name, boolean ok, String detail) {
        checks++;
        if (!ok) failures++;
        System.out.printf("  [%s] %-58s %s%n", ok ? "PASS" : "FAIL", name, detail);
    }

    public static void main(String[] args) {
        long t0 = System.nanoTime();
        observerTests();
        flywheelTests();
        turretTests();
        ballisticsTests();
        aimTests();
        sysIdTest();
        systemTest(false);
        systemTest(true);
        allocationAndTimingTest();
        System.out.printf("%n%d checks, %d failures (%.1f s)%n", checks, failures, (System.nanoTime() - t0) * 1e-9);
        if (failures > 0) System.exit(1);
    }

    // ------------------------------------------------------------------------------------------ observer
    static void observerTests() {
        System.out.println("\n== Extended State Observer ==");
        ExtendedStateObserver eso = new ExtendedStateObserver(40);
        // truth: accel = 50 + unknown disturbance -120 rad/s², quantized position, jittery dt
        Random rnd = new Random(1);
        double pos = 0, vel = 0, t = 0;
        double knownAccel = 50, dist = -120;
        double maxAbsErrLate = 0, distSum = 0;
        int distN = 0;
        for (int k = 0; k < 400; k++) {
            double dt = 0.008 + 0.008 * rnd.nextDouble();
            for (int s = 0; s < 10; s++) {
                vel += (knownAccel + dist) * dt / 10;
                pos += vel * dt / 10;
            }
            t += dt;
            double meas = Math.floor(pos / (2 * Math.PI / 28)) * (2 * Math.PI / 28);
            eso.update(meas, knownAccel, dt);
            if (t > 1.0) maxAbsErrLate = Math.max(maxAbsErrLate, Math.abs(eso.velocity() - vel));
            if (t > 3.8) { distSum += eso.disturbance(); distN++; }
        }
        // Quantization floor: σ_q = q/√12 ≈ 0.065 rad → velocity noise ≈ σ_q·wo·O(1) ≈ 3–6 rad/s at wo = 40,
        // independent of speed, so the bound is absolute (≈1.5% at a 400 rad/s shooting speed).
        check("ESO velocity error after 1 s (28 cpr, jitter): < 6.5 rad/s", maxAbsErrLate < 6.5,
                String.format("max %.2f rad/s (theory floor ≈ 3–6)", maxAbsErrLate));
        double distMean = distSum / distN;
        check("ESO disturbance estimate (1 s mean) within 10%", Math.abs(distMean - dist) < 12,
                String.format("mean %.1f vs true %.1f rad/s²", distMean, dist));
        // pole placement exactness for a few (wo, dt)
        boolean polesOk = true;
        for (double[] c : new double[][]{{40, 0.01}, {120, 0.02}, {5, 0.002}}) {
            double T = c[1], b = Math.exp(-c[0] * T);
            double l1 = ExtendedStateObserver.gainL1(c[0], T), l2 = ExtendedStateObserver.gainL2(c[0], T),
                    l3 = ExtendedStateObserver.gainL3(c[0], T);
            double[][] P = {{1, T, T * T / 2}, {0, 1, T}, {0, 0, 1}};
            double[] L = {l1, l2, l3};
            double[][] M = new double[3][3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) M[i][j] = P[i][j] - L[i] * P[0][j];
            double tr = M[0][0] + M[1][1] + M[2][2];
            double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
            polesOk &= Math.abs(tr - 3 * b) < 1e-9 && Math.abs(det - b * b * b) < 1e-9;
        }
        check("ESO discrete poles exactly at exp(-wo·dt) (triple)", polesOk, "");
    }

    // ------------------------------------------------------------------------------------------ flywheel
    static final class FwResult {
        double spinupTime = Double.NaN, rmsErr, maxDev, recoveryWorst;
        int detectedShots;
    }

    /** Controller tuned with deliberately imperfect "identified" values (tests robustness). */
    static FlywheelController.Config fwConfig() {
        FlywheelController.Config c = new FlywheelController.Config();
        c.model = new MotorFeedforward(0.30, 0.0185, 0.0150, 2.0); // truth ≈ 0.33, 0.0189, 0.0166 (and J +20% below)
        return c;
    }

    static FwResult runFlywheel(double voc, double loadStepAt, double loadAmps, double[] shotTimes,
                                boolean anticipate, double contactLate, boolean glitch) {
        FlywheelSim sim = new FlywheelSim();
        sim.batteryOpenCircuit = voc;
        sim.J *= 1.2; // model mismatch: real wheel heavier than the controller believes
        FlywheelController ctl = new FlywheelController(fwConfig());
        VoltageCompensator vc = new VoltageCompensator(0.05, 0.05, 12.5);
        Random rnd = new Random(7);
        double target = 420; // ≈ 4010 rpm
        FwResult res = new FwResult();
        double t = 0, pending = 0, pendingAt = -1;
        double sumSq = 0; int n = 0;
        int shotIdx = 0;
        double[] contactAt = new double[shotTimes.length];
        double[] recover = new double[shotTimes.length];
        java.util.Arrays.fill(contactAt, -1);
        java.util.Arrays.fill(recover, Double.NaN);
        double inBandSince = -1;
        ctl.setTarget(target);
        while (t < 4.0) {
            double dt = 0.008 + 0.007 * rnd.nextDouble();
            double loopStart = t;
            double meas = sim.measuredPositionRad();
            if (glitch && Math.abs(t - 2.0) < 0.006) meas += 200; // one corrupt reading
            if (vc.shouldSample(t)) vc.addSample(sim.terminalVolts + 0.02 * rnd.nextGaussian(), t);
            // feeder
            if (shotIdx < shotTimes.length && t >= shotTimes[shotIdx]) {
                if (anticipate) ctl.notifyFeed();
                contactAt[shotIdx] = t + 0.08 + contactLate;
                shotIdx++;
            }
            double u = ctl.update(meas, t, vc.volts());
            pending = vc.toPower(u);
            pendingAt = loopStart + 0.002; // I/O write latency
            double end = t + dt;
            while (t < end) {
                double h = 0.0002;
                if (pendingAt >= 0 && t >= pendingAt) { sim.setPower(pending); pendingAt = -1; }
                for (int i = 0; i < contactAt.length; i++) {
                    if (contactAt[i] >= 0 && t >= contactAt[i]) { sim.ballContact(t, 2.5, 0.04); contactAt[i] = -2; }
                }
                if (loadStepAt > 0 && t >= loadStepAt) sim.otherLoadCurrent = loadAmps;
                sim.step(t, h);
                t += h;
            }
            double err = sim.omega - target;
            boolean inBand = Math.abs(err) <= 0.02 * target;
            if (inBand && inBandSince < 0) inBandSince = t;
            if (!inBand) inBandSince = -1;
            if (Double.isNaN(res.spinupTime) && inBandSince >= 0 && t - inBandSince > 0.1) res.spinupTime = inBandSince;
            if (t > 1.5 && shotTimes.length == 0) {
                sumSq += err * err; n++;
                res.maxDev = Math.max(res.maxDev, Math.abs(err));
            }
            for (int i = 0; i < shotTimes.length; i++) {
                double c = shotTimes[i] + 0.08 + contactLate;
                if (t > c && Double.isNaN(recover[i]) && inBandSince >= 0 && inBandSince > c && t - inBandSince > 0.03) {
                    recover[i] = inBandSince - c;
                }
            }
        }
        res.rmsErr = n > 0 ? Math.sqrt(sumSq / n) : Double.NaN;
        for (double r : recover) res.recoveryWorst = Math.max(res.recoveryWorst, Double.isNaN(r) ? 9 : r);
        res.detectedShots = ctl.shotCount();
        return res;
    }

    /** Physical lower bound: time to reach 98% of 420 rad/s with full available voltage the whole way. */
    static double spinupBound(double voc) {
        FlywheelSim sim = new FlywheelSim();
        sim.batteryOpenCircuit = voc;
        sim.J *= 1.2;
        sim.setPower(0.97);
        double t = 0;
        while (sim.omega < 0.98 * 420 && t < 5) { sim.step(t, 0.0002); t += 0.0002; }
        return t;
    }

    static void flywheelTests() {
        System.out.println("\n== Flywheel (ADRC + feedforward, 28 cpr, jitter, I/O delay, +20% inertia mismatch) ==");
        FwResult a = runFlywheel(12.8, -1, 0, new double[0], true, 0, false);
        double bA = spinupBound(12.8);
        check("F1 spin-up 0→420 rad/s @12.8 V vs physical minimum", a.spinupTime <= 1.10 * bA + 0.05,
                String.format("%.3f s, physics bound %.3f s (req ≤ 1.1×bound + 50 ms)", a.spinupTime, bA));
        check("F2 steady RMS speed error @12.8 V", a.rmsErr / 420 <= 0.01,
                String.format("%.2f%% (max %.2f%%) (req ≤ 1%%)", 100 * a.rmsErr / 420, 100 * a.maxDev / 420));
        FwResult b = runFlywheel(11.3, -1, 0, new double[0], true, 0, false);
        double bB = spinupBound(11.3);
        check("F1/F4 spin-up @11.3 V (sagging battery) vs physical minimum", b.spinupTime <= 1.10 * bB + 0.05,
                String.format("%.3f s, physics bound %.3f s", b.spinupTime, bB));
        check("F4 steady RMS error @11.3 V", b.rmsErr / 420 <= 0.01, String.format("%.2f%%", 100 * b.rmsErr / 420));
        FwResult c = runFlywheel(12.8, 2.0, 35, new double[0], true, 0, false);
        check("F4 drivetrain load step 35 A (≈2 V sag): max deviation", c.maxDev / 420 <= 0.02,
                String.format("%.2f%% (RMS %.2f%%)", 100 * c.maxDev / 420, 100 * c.rmsErr / 420));
        double[] shots = {1.8, 2.2, 2.6, 3.0};
        FwResult d = runFlywheel(12.6, -1, 0, shots, false, 0, false);
        FwResult e = runFlywheel(12.6, -1, 0, shots, true, 0, false);
        FwResult f = runFlywheel(12.6, -1, 0, shots, true, 0.02, false);
        check("F3 recovery to 2% after 2.5 J shot (reactive only)", d.recoveryWorst <= 0.25,
                String.format("worst %.0f ms", 1000 * d.recoveryWorst));
        check("F3 recovery with anticipation + energy banking: ≥20% faster", e.recoveryWorst <= 0.8 * d.recoveryWorst,
                String.format("worst %.0f ms (vs %.0f reactive)", 1000 * e.recoveryWorst, 1000 * d.recoveryWorst));
        check("F3 anticipation robust to 20 ms contact-timing error", f.recoveryWorst <= 0.25,
                String.format("worst %.0f ms", 1000 * f.recoveryWorst));
        check("Shot detection from disturbance estimate (4 shots)", d.detectedShots == 4,
                String.format("detected %d", d.detectedShots));
        // Unplugged encoder: reading frozen at a constant → stall fault, output 0, cleared by target 0
        {
            FlywheelController ctl = new FlywheelController(fwConfig());
            ctl.setTarget(420);
            double t = 0, faultAt = Double.NaN, u = 1;
            while (t < 2.0) {
                u = ctl.update(123.0, t, 12.5);
                if (ctl.isFaulted() && Double.isNaN(faultAt)) faultAt = t;
                t += 0.01;
            }
            ctl.setTarget(0);
            boolean cleared = !ctl.isStalled();
            check("S2 dead flywheel encoder → stall fault, 0 V, clearable", faultAt <= 0.75 && u == 0 && cleared,
                    String.format("fault after %.2f s", faultAt));
        }
        FwResult g = runFlywheel(12.8, -1, 0, new double[0], true, 0, true);
        check("S2 single corrupt encoder sample: speed error bounded", g.maxDev / 420 <= 0.02,
                String.format("max dev %.2f%%", 100 * g.maxDev / 420));
    }

    // ------------------------------------------------------------------------------------------ turret
    static TurretController.Config turretConfig() {
        TurretController.Config c = new TurretController.Config();
        // "identified" values ~20–25% off the simulator truth (kS≈0.05, kV≈0.32, kA≈0.0036)
        c.model = new MotorFeedforward(0.045, 0.30, 0.0045, 0.05);
        return c;
    }

    interface TurretScenario { void apply(double t, double[] out); } // out: {relTargetAngle, relRate, robotYawAccel}

    static final class TurretRun {
        double settle = Double.NaN, overshoot, rmsErr, maxErr, minAngleSeen = 1e9, maxAngleSeen = -1e9;
        int signChangesAtRest;
        double restErr;
        boolean outputZeroOnFault;
    }

    static TurretRun runTurret(double startAngle, double duration, TurretScenario sc, double preload,
                               double settleWindowStart, double measureFrom, boolean faultAt2s) {
        TurretSim sim = new TurretSim();
        sim.reset(startAngle);
        TurretController.Config cfg = turretConfig();
        cfg.preloadPower = preload;
        TurretController ctl = new TurretController(cfg);
        Random rnd = new Random(3);
        double t = 0;
        double[] tgt = new double[3];
        TurretRun r = new TurretRun();
        double sumSq = 0; int n = 0;
        double inBandSince = -1;
        double lastSign = 0;
        double pendL = 0, pendR = 0, pendAt = -1;
        while (t < duration) {
            double dt = 0.008 + 0.007 * rnd.nextDouble();
            sc.apply(t, tgt);
            sim.robotYawAccel = tgt[2];
            ctl.setTarget(tgt[0], tgt[1]);
            double meas = sim.measuredAngle();
            if (faultAt2s && t > 2.0) meas = Double.NaN;
            ctl.update(meas, t);
            if (faultAt2s && t > 2.0 && t < 2.05) {
                r.outputZeroOnFault = ctl.leftPower() == 0 && ctl.rightPower() == 0 && ctl.isFaulted();
            }
            pendL = ctl.leftPower(); pendR = ctl.rightPower(); pendAt = t + 0.002;
            double end = t + dt;
            while (t < end) {
                if (pendAt >= 0 && t >= pendAt) { sim.setPowers(pendL, pendR); pendAt = -1; }
                sim.step(0.0001);
                t += 0.0001;
            }
            double err = MathUtil.wrapPi(tgt[0] - sim.ring);
            r.minAngleSeen = Math.min(r.minAngleSeen, sim.ring);
            r.maxAngleSeen = Math.max(r.maxAngleSeen, sim.ring);
            if (t >= settleWindowStart) {
                boolean in = Math.abs(err) <= Math.toRadians(1.0);
                if (in && inBandSince < 0) inBandSince = t;
                if (!in) inBandSince = -1;
                if (Double.isNaN(r.settle) && inBandSince >= 0 && t - inBandSince > 0.2) r.settle = inBandSince - settleWindowStart;
                r.overshoot = Math.max(r.overshoot, -Math.signum(tgt[0] - startAngle) * err);
            }
            if (t >= measureFrom) {
                sumSq += err * err; n++;
                r.maxErr = Math.max(r.maxErr, Math.abs(err));
                double s = Math.signum(ctl.appliedPower());
                if (s != 0 && lastSign != 0 && s != lastSign) r.signChangesAtRest++;
                if (s != 0) lastSign = s;
                r.restErr = err;
            }
        }
        r.rmsErr = n > 0 ? Math.sqrt(sumSq / n) : Double.NaN;
        return r;
    }

    static void turretTests() {
        System.out.println("\n== Turret (ADRC + time-optimal reference, deadband, 1° backlash, model mismatch) ==");
        double ninety = Math.toRadians(90);
        TurretRun step = runTurret(0, 3.0, (t, o) -> { o[0] = t > 0.2 ? ninety : 0; o[1] = 0; o[2] = 0; },
                0, 0.2, 2.0, false);
        check("T1 90° step: settle within ±1°", step.settle <= 0.8,
                String.format("%.3f s, overshoot %.2f° (req ≤ 0.8 s)", step.settle, Math.toDegrees(step.overshoot)));
        check("T3 no hunting at rest (output sign changes over 1 s)", step.signChangesAtRest == 0,
                String.format("%d changes, rest error %.2f°", step.signChangesAtRest, Math.toDegrees(step.restErr)));

        for (double rateDeg : new double[]{90, 150}) {
            double w = Math.toRadians(rateDeg);
            // Robot ramps its spin up at 8 rad/s² while the turret holds a fixed field direction
            // (relative target = 170° − heading). Duration keeps the travel within the ±200° soft limits
            // (a continuous spin past 360° must unwind, which is a different, expected behaviour).
            double start = Math.toRadians(170);
            double duration = 0.5 * (w / 8.0) + Math.toRadians(345) / w; // heading ends at 345° → target −175°
            TurretRun spin = runTurret(start, duration, (t, o) -> {
                double ramp = w / 8.0;
                double heading, omega, alpha;
                if (t < ramp) { omega = 8.0 * t; heading = 4.0 * t * t; alpha = 8.0; }
                else { omega = w; heading = 4.0 * ramp * ramp + w * (t - ramp); alpha = 0; }
                o[0] = start - heading; o[1] = -omega; o[2] = alpha;
            }, 0, 99, 0.3, false);
            // 90°/s ≈ 52% of servo speed: precise tracking required. 150°/s ≈ 87%: the feedforward alone needs
            // ~91% authority, so the requirement is graceful degradation (bounded, no divergence); the fire
            // gate blocks shots while the error exceeds the tolerance.
            boolean ok = rateDeg <= 90 ? Math.toDegrees(spin.maxErr) <= 1.5 : Math.toDegrees(spin.maxErr) <= 5.0;
            check(String.format("T2 hold field heading while robot spins %.0f°/s", rateDeg), ok,
                    String.format("RMS %.2f°, max %.2f° (req ≤ %s)", Math.toDegrees(spin.rmsErr), Math.toDegrees(spin.maxErr),
                            rateDeg <= 90 ? "1.5°" : "5° near saturation"));
        }

        TurretRun pre = runTurret(0, 3.0, (t, o) -> { o[0] = t > 0.2 ? ninety : 0; o[1] = 0; o[2] = 0; },
                0.12, 0.2, 2.0, false);
        check("T1 with anti-backlash preload 0.12", pre.settle <= 0.8 && pre.signChangesAtRest == 0,
                String.format("settle %.3f s, rest error %.2f°", pre.settle, Math.toDegrees(pre.restErr)));

        // routing near the ±180° seam: start at 170°, target sweeps 170° → 190° (= −170°) and back
        TurretRun seam = runTurret(Math.toRadians(170), 4.0, (t, o) -> {
            double a = Math.toRadians(170 + 20 * Math.sin(Math.PI * t / 2));
            o[0] = MathUtil.wrapPi(a); o[1] = Math.toRadians(20 * Math.PI / 2 * Math.cos(Math.PI * t / 2)); o[2] = 0;
        }, 0, 99, 0.5, false);
        check("Routing across ±180°: never takes the long way", Math.toDegrees(seam.minAngleSeen) > 140,
                String.format("min angle visited %.1f°, max %.1f°", Math.toDegrees(seam.minAngleSeen), Math.toDegrees(seam.maxAngleSeen)));

        TurretRun fault = runTurret(0, 2.5, (t, o) -> { o[0] = ninety; o[1] = 0; o[2] = 0; }, 0, 99, 99, true);
        check("S2 turret sensor loss → outputs 0 within one loop", fault.outputZeroOnFault, "");
        {
            TurretController ctl = new TurretController(turretConfig());
            double t = 0, faultAt = Double.NaN;
            while (t < 2.0) {
                ctl.setTarget(ninety, 0);
                ctl.update(0.0, t); // frozen reading (encoder unplugged)
                if (ctl.isFaulted() && Double.isNaN(faultAt)) faultAt = t;
                t += 0.01;
            }
            check("S2 frozen turret encoder → stuck fault, servos off", faultAt <= 0.8 && ctl.leftPower() == 0
                    && ctl.rightPower() == 0, String.format("fault after %.2f s", faultAt));
        }
    }

    // ------------------------------------------------------------------------------------------ ballistics
    static Ballistics.Params ballParams() {
        return new Ballistics.Params();
    }

    static HoodModel.Config hoodConfig() {
        return new HoodModel.Config();
    }

    static ShotTable buildTable(Ballistics b) {
        AimSolver.Config ac = new AimSolver.Config();
        HiveTarget.Config hc = new HiveTarget.Config();
        HoodModel hood = new HoodModel(hoodConfig());
        ShooterCore.Config sc = new ShooterCore.Config();
        double maxExit = sc.exitEfficiency * sc.wheelRadiusM * sc.maxFlywheelSpeed;
        return new ShotTable(b, new ShotTable.Config(),
                (hc.openingCenterZ - ac.exitHeight) * MathUtil.IN_TO_M, ac.exitHeight * MathUtil.IN_TO_M,
                hood.minAngle(), hood.maxAngle(), maxExit);
    }

    static void ballisticsTests() {
        System.out.println("\n== Ballistics ==");
        Ballistics.Params vac = new Ballistics.Params();
        vac.dragCoefficient = 0;
        vac.liftCoefficient = 0;
        Ballistics bv = new Ballistics(vac);
        double v = 8, th = Math.toRadians(50), x = 2.0;
        double analytic = x * Math.tan(th) - 9.80665 * x * x / (2 * v * v * Math.cos(th) * Math.cos(th));
        double z = bv.heightAt(v, th, x, 1);
        check("RK4 vs analytic (no air): height at 2 m", Math.abs(z - analytic) < 1e-3,
                String.format("err %.2e m", Math.abs(z - analytic)));

        Ballistics b = new Ballistics(ballParams());
        long t0 = System.nanoTime();
        ShotTable table = buildTable(b);
        double ms = (System.nanoTime() - t0) * 1e-6;
        int validCount = 0;
        for (int i = 0; i < table.size(); i++) if (table.validAt(i)) validCount++;
        check("Shot table builds (init time on PC)", table.hasAnySolution(),
                String.format("%.0f ms, %d/%d distances valid, range %.2f–%.2f m", ms, validCount, table.size(),
                        table.minValidDistance(), table.maxValidDistance()));
        System.out.println("     d(m)  angle(°)  exit(m/s)  flywheel(rpm)  tof(s)");
        ShooterCore.Config sc = new ShooterCore.Config();
        for (int i = 0; i < table.size(); i += 4) {
            if (!table.validAt(i)) continue;
            double w = table.speedAt(i) / (sc.exitEfficiency * sc.wheelRadiusM);
            System.out.printf("     %.2f   %6.1f    %6.2f      %6.0f       %.2f%n", table.distanceAt(i),
                    Math.toDegrees(table.angleAt(i)), table.speedAt(i), w * 60 / (2 * Math.PI), table.tofAt(i));
        }
        boolean coversField = table.minValidDistance() <= 0.8 && table.maxValidDistance() >= 3.5;
        check("Table covers 0.8–3.5 m (HIVE at centre; field corner ≈ 2.9 m)", coversField,
                String.format("%.2f–%.2f m", table.minValidDistance(), table.maxValidDistance()));

        // compensation consistency: fly the compensated angle at 92% speed and check the height at the target
        double worst = 0;
        Ballistics.Crossing c = new Ballistics.Crossing();
        double floor = new AimSolver.Config().exitHeight * MathUtil.IN_TO_M;
        int compPoints = 0, compValid = 0;
        HoodModel hm = new HoodModel(hoodConfig());
        for (double d = 1.03; d <= 3.4; d += 0.17) {
            ShotTable.Solution s = table.nominal(d, new ShotTable.Solution());
            if (!s.valid) continue;
            double peakNom = b.peakAngle(s.exitSpeed, d, hm.minAngle(), hm.maxAngle(), floor);
            boolean increasing = s.launchAngle < peakNom;
            for (double f : new double[]{0.95, 1.05}) {
                double vc = s.exitSpeed * f;
                // ground truth: direct physics solve on the same branch within the hood range
                double pk = b.peakAngle(vc, d, hm.minAngle(), hm.maxAngle(), floor);
                double exact = b.solveAngle(vc, d, table.targetHeightAboveExit(),
                        increasing ? hm.minAngle() : pk, increasing ? pk : hm.maxAngle(), increasing, floor);
                if (Double.isNaN(exact)) continue; // physically impossible within the hood range: not a table defect
                compPoints++;
                double a = table.compensatedAngle(d, vc);
                if (Double.isNaN(a)) continue;
                compValid++;
                b.fly(s.exitSpeed * f, a, d, floor, c);
                worst = Math.max(worst, Math.abs(c.z - table.targetHeightAboveExit()));
            }
        }
        check("Speed-adaptive hood (±5% speed): table matches direct physics solve", worst < 0.02 && compValid >= 0.95 * compPoints,
                String.format("worst %.1f mm (req < 20), table covers %d/%d physically solvable points", worst * 1000, compValid, compPoints));

        // cache round trip: identical lookups after write/read; a changed parameter invalidates the cache
        boolean same = false, rejected = false;
        try {
            java.io.ByteArrayOutputStream bos = new java.io.ByteArrayOutputStream();
            long sig = 42L;
            table.write(new java.io.DataOutputStream(bos), sig);
            ShotTable back = ShotTable.read(new java.io.DataInputStream(new java.io.ByteArrayInputStream(bos.toByteArray())),
                    new ShotTable.Config(), sig);
            ShotTable stale = ShotTable.read(new java.io.DataInputStream(new java.io.ByteArrayInputStream(bos.toByteArray())),
                    new ShotTable.Config(), sig + 1);
            same = back != null;
            for (double d = 0.75; same && d < 4.3; d += 0.13) {
                ShotTable.Solution s1 = table.nominal(d, new ShotTable.Solution()), s2 = back.nominal(d, new ShotTable.Solution());
                same = s1.valid == s2.valid && (!s1.valid || (s1.launchAngle == s2.launchAngle && s1.exitSpeed == s2.exitSpeed
                        && Double.compare(table.compensatedAngle(d, s1.exitSpeed * 0.97), back.compensatedAngle(d, s1.exitSpeed * 0.97)) == 0));
            }
            rejected = stale == null;
            Ballistics.Params changed = ballParams();
            changed.liftCoefficient += 0.01;
            rejected &= ShotTable.signature(ballParams(), new ShotTable.Config(), 1, 0.3, 0.7, 1.4, 9)
                    != ShotTable.signature(changed, new ShotTable.Config(), 1, 0.3, 0.7, 1.4, 9);
        } catch (java.io.IOException e) {
            same = false;
        }
        check("Shot-table cache: exact round trip, stale cache rejected", same && rejected, "");

        // the robustness of the chosen arc vs. a naive 45°-ish choice
        double d = 2.5;
        ShotTable.Solution s = table.nominal(d, new ShotTable.Solution());
        double dzdv = (b.heightAt(s.exitSpeed * 1.02, s.launchAngle, d, floor) - b.heightAt(s.exitSpeed * 0.98, s.launchAngle, d, floor)) / 2;
        System.out.printf("     at 2.5 m: chosen %.1f°, ±2%% speed → ±%.1f cm vertical miss%n",
                Math.toDegrees(s.launchAngle), 100 * Math.abs(dzdv));
    }

    // ------------------------------------------------------------------------------------------ aim
    /** 3D ball flight (drag + lift in the vertical plane of the velocity) for end-to-end aim checks. */
    static double[] fly3d(Ballistics.Params p, double[] pos, double[] vel, double targetZ) {
        double k = 0.5 * p.airDensity * Math.PI * 0.25 * p.diameterM * p.diameterM / p.massKg;
        double x = pos[0], y = pos[1], z = pos[2], vx = vel[0], vy = vel[1], vz = vel[2];
        double h = 0.001, t = 0;
        boolean wasAbove = z > targetZ;
        while (t < 3) {
            double s = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double hs = Math.sqrt(vx * vx + vy * vy);
            // lift ⟂ v in the vertical plane, pointing "up": n = (−vz·vh/|vh|, |vh|) normalized
            double nx = hs > 1e-9 ? -vz * vx / hs : 0, ny = hs > 1e-9 ? -vz * vy / hs : 0, nz = hs;
            double nn = Math.sqrt(nx * nx + ny * ny + nz * nz);
            nx /= nn; ny /= nn; nz /= nn;
            double ax = -k * p.dragCoefficient * s * vx + k * p.liftCoefficient * s * s * nx;
            double ay = -k * p.dragCoefficient * s * vy + k * p.liftCoefficient * s * s * ny;
            double az = -p.gravity - k * p.dragCoefficient * s * vz + k * p.liftCoefficient * s * s * nz;
            double pz = z;
            vx += ax * h; vy += ay * h; vz += az * h;
            x += vx * h; y += vy * h; z += vz * h; t += h;
            if (wasAbove && z <= targetZ && vz < 0) {
                double f = (pz - targetZ) / (pz - z);
                return new double[]{x - vx * h * (1 - f), y - vy * h * (1 - f), t};
            }
            if (z > targetZ) wasAbove = true;
        }
        return null;
    }

    static double aimMiss(RobotState s, HiveTarget tgt, ShotTable table, AimSolver solver, Ballistics.Params bp,
                          boolean compensate) {
        AimSolver.Aim aim = solver.solve(s, tgt, table, s.timestamp, new AimSolver.Aim());
        if (!aim.valid) return 998;
        AimSolver.Config ac = solver.config();
        double lat = ac.latency;
        double h = s.heading + s.omega * lat;
        double px = s.x + s.vx * lat, py = s.y + s.vy * lat;
        double c = Math.cos(h), sn = Math.sin(h);
        double rx = c * ac.turretForward - sn * ac.turretLeft, ry = sn * ac.turretForward + c * ac.turretLeft;
        double lvx = compensate ? s.vx - s.omega * ry : 0, lvy = compensate ? s.vy + s.omega * rx : 0;
        double dir = h + aim.turretAngle;
        double v = aim.shot.exitSpeed, el = aim.shot.launchAngle;
        double[] pos = {(px + rx) * MathUtil.IN_TO_M, (py + ry) * MathUtil.IN_TO_M, ac.exitHeight * MathUtil.IN_TO_M};
        double[] vel = {v * Math.cos(el) * Math.cos(dir) + (s.vx - s.omega * ry) * MathUtil.IN_TO_M,
                v * Math.cos(el) * Math.sin(dir) + (s.vy + s.omega * rx) * MathUtil.IN_TO_M,
                v * Math.sin(el)};
        double[] hit = fly3d(bp, pos, vel, tgt.z() * MathUtil.IN_TO_M);
        if (hit == null) return 999;
        return Math.hypot(hit[0] * MathUtil.M_TO_IN - tgt.x(), hit[1] * MathUtil.M_TO_IN - tgt.y());
    }

    static void aimTests() {
        System.out.println("\n== Aim solver (3D flight check) ==");
        Ballistics.Params bp = ballParams();
        Ballistics b = new Ballistics(bp);
        ShotTable table = buildTable(b);
        AimSolver solver = new AimSolver(new AimSolver.Config());
        HiveTarget tgt = new HiveTarget(new HiveTarget.Config(), HiveTarget.Alliance.RED, HiveTarget.UpCell.AUDIENCE_SIDE);

        double worstStill = 0;
        for (double[] p : new double[][]{{-30, -70}, {-60, -40}, {20, -90}, {-90, -100}, {40, 20}}) {
            RobotState still = new RobotState().set(p[0], p[1], Math.toRadians(30), 0, 0, 0, 0);
            worstStill = Math.max(worstStill, aimMiss(still, tgt, table, solver, bp, true));
        }
        check("Stationary shots (5 field positions) land at the cell centre", worstStill < 1.5,
                String.format("worst miss %.2f in", worstStill));

        RobotState moving = new RobotState().set(-60, -60, Math.toRadians(-20), 30, 20, Math.toRadians(60), 0);
        double m1 = aimMiss(moving, tgt, table, solver, bp, true);
        AimSolver.Config naiveCfg = new AimSolver.Config();
        naiveCfg.iterations = 0; // no lead
        RobotState zeroV = new RobotState().set(moving.x, moving.y, moving.heading, 0, 0, 0, 0);
        AimSolver naive = new AimSolver(naiveCfg);
        // naive: aim as if stationary, but the ball inherits the robot velocity
        AimSolver.Aim na = naive.solve(zeroV, tgt, table, 0, new AimSolver.Aim());
        double ballDir = moving.heading + na.turretAngle;
        double[] pos = {(moving.x + Math.cos(moving.heading) * -2.82) * MathUtil.IN_TO_M,
                (moving.y + Math.sin(moving.heading) * -2.82) * MathUtil.IN_TO_M, 12.4 * MathUtil.IN_TO_M};
        double[] vel = {na.shot.exitSpeed * Math.cos(na.shot.launchAngle) * Math.cos(ballDir) + 30 * MathUtil.IN_TO_M,
                na.shot.exitSpeed * Math.cos(na.shot.launchAngle) * Math.sin(ballDir) + 20 * MathUtil.IN_TO_M,
                na.shot.exitSpeed * Math.sin(na.shot.launchAngle)};
        double[] hit = fly3d(bp, pos, vel, tgt.z() * MathUtil.IN_TO_M);
        double mNaive = hit == null ? 999 : Math.hypot(hit[0] * MathUtil.M_TO_IN - tgt.x(), hit[1] * MathUtil.M_TO_IN - tgt.y());
        check("Shoot on the move (36 in/s, 60°/s): lead-compensated miss", m1 < 4.0,
                String.format("miss %.1f in (uncompensated %.1f in; allowed ≈ 8.6)", m1, mNaive));
    }

    // ------------------------------------------------------------------------------------------ sysid
    static void sysIdTest() {
        System.out.println("\n== System identification (centered-difference OLS on the characterization profile) ==");
        FlywheelSim sim = new FlywheelSim();
        Random rnd = new Random(5);
        int cap = 6000;
        double[] ts = new double[cap], ps = new double[cap], us = new double[cap];
        int n = 0;
        double t = 0;
        // Same profile as the Flywheel Characterization OpMode: quasistatic ramp 0.5 V/s, then voltage steps.
        while (t < 34 && n < cap) {
            double dt = 0.008 + 0.007 * rnd.nextDouble();
            double volts = t < 22 ? 0.5 * t : ((((int) ((t - 22) / 1.5)) % 2 == 0) ? 11 : 4);
            double measuredBattery = sim.terminalVolts + 0.02 * rnd.nextGaussian();
            double power = volts / measuredBattery;
            sim.setPower(power);
            ts[n] = t; ps[n] = sim.measuredPositionRad(); us[n] = power * measuredBattery; n++;
            double end = t + dt;
            while (t < end) { sim.step(t, 0.0002); t += 0.0002; }
        }
        SysIdFitter.Result r = SysIdFitter.fit(ts, ps, us, n, 4, 20);
        double kS = sim.R * sim.coulomb / sim.Kt, kV = sim.Ke + sim.R * sim.viscous / sim.Kt, kA = sim.J * sim.R / sim.Kt;
        boolean ok = r.ok && Math.abs(r.kV - kV) / kV < 0.03 && Math.abs(r.kA - kA) / kA < 0.10 && Math.abs(r.kS - kS) < 0.08;
        check("Fit recovers kS, kV, kA (28 cpr, jitter, sag)", ok,
                String.format("kS %.3f/%.3f kV %.4f/%.4f kA %.4f/%.4f R²=%.4f", r.kS, kS, r.kV, kV, r.kA, kA, r.rSquared));
    }

    // ------------------------------------------------------------------------------------------ full system
    static void systemTest(boolean moving) {
        System.out.println(moving ? "\n== Full system: driving 30 in/s + rotating while firing ==" : "\n== Full system: stationary, 4-ball burst ==");
        Ballistics.Params bp = ballParams();
        Ballistics b = new Ballistics(bp);
        ShotTable table = buildTable(b);
        ShooterCore.Config scfg = new ShooterCore.Config();
        FlywheelController fw = new FlywheelController(fwConfig());
        TurretController tc = new TurretController(turretConfig());
        HoodModel hood = new HoodModel(hoodConfig());
        AimSolver solver = new AimSolver(new AimSolver.Config());
        HiveTarget tgt = new HiveTarget(new HiveTarget.Config(), HiveTarget.Alliance.RED, HiveTarget.UpCell.AUDIENCE_SIDE);
        ShooterCore core = new ShooterCore(scfg, fw, tc, hood, solver, tgt, table);
        core.setMode(ShooterCore.Mode.TRACK);

        FlywheelSim fsim = new FlywheelSim();
        fsim.J *= 1.2;
        TurretSim tsim = new TurretSim();
        HoodModel hoodTruth = new HoodModel(hoodConfig()); // "true" hood motion (same model, slower servo)
        hoodTruth.config().slewRateDegPerSec = 220;
        VoltageCompensator vc = new VoltageCompensator(0.05, 0.05, 12.5);
        RobotState rs = new RobotState();
        Random rnd = new Random(11);

        double t = 0, x = -50, y = -80, hdg = Math.toRadians(90);
        double vx = moving ? 30 : 0, vy = moving ? 6 : 0, w = moving ? Math.toRadians(40) : 0;
        int fired = 0, hits = 0;
        double firstShot = Double.NaN, lastFeed = -1;
        double worstMiss = 0;
        double pendF = 0, pendL = 0, pendR = 0, pendAt = -1;
        java.util.List<double[]> inFlight = new java.util.ArrayList<>();
        java.util.EnumMap<ShooterCore.Readiness, Integer> reasons = new java.util.EnumMap<>(ShooterCore.Readiness.class);
        double stopAt = 4.5;
        while (t < stopAt) {
            if (fired >= 4) stopAt = Math.min(stopAt, lastFeed + 0.3); // let the last ball reach the wheel
            double dt = 0.008 + 0.007 * rnd.nextDouble();
            rs.set(x, y, hdg, vx, vy, w, t);
            if (vc.shouldSample(t)) vc.addSample(fsim.terminalVolts + 0.02 * rnd.nextGaussian(), t);
            core.update(rs, fsim.measuredPositionRad(), tsim.measuredAngle(), vc.volts(), t);
            if (!Double.isNaN(core.hoodServoPosition())) hoodTruth.command(hood.commandedAngle());
            reasons.merge(core.readiness(), 1, Integer::sum);
            if (fired < 4 && core.canFire() && t - lastFeed > 0.25) {
                core.notifyFired();
                lastFeed = t;
                fired++;
                if (Double.isNaN(firstShot)) firstShot = t;
                inFlight.add(new double[]{t + 0.08}); // contact time
            }
            pendF = vc.toPower(core.flywheelVolts()); pendL = core.turretLeftPower(); pendR = core.turretRightPower(); pendAt = t + 0.002;
            double end = t + dt;
            while (t < end) {
                double hstep = 0.0002;
                if (pendAt >= 0 && t >= pendAt) { fsim.setPower(pendF); tsim.setPowers(pendL, pendR); pendAt = -1; }
                for (double[] ball : inFlight) {
                    if (ball[0] > 0 && t >= ball[0]) {
                        // ball leaves with the TRUE wheel speed, hood angle and turret direction at contact
                        double exit = scfg.exitEfficiency * scfg.wheelRadiusM * fsim.omega;
                        double el = hoodTruth.estimatedAngle();
                        double dir = hdg + tsim.ring;
                        AimSolver.Config ac = solver.config();
                        double c = Math.cos(hdg), sn = Math.sin(hdg);
                        double rx = c * ac.turretForward, ry = sn * ac.turretForward;
                        double[] pos = {(x + rx) * MathUtil.IN_TO_M, (y + ry) * MathUtil.IN_TO_M, ac.exitHeight * MathUtil.IN_TO_M};
                        double[] vel = {exit * Math.cos(el) * Math.cos(dir) + (vx - w * ry) * MathUtil.IN_TO_M,
                                exit * Math.cos(el) * Math.sin(dir) + (vy + w * rx) * MathUtil.IN_TO_M, exit * Math.sin(el)};
                        double[] hit = fly3d(bp, pos, vel, tgt.z() * MathUtil.IN_TO_M);
                        double miss = hit == null ? 999 : Math.hypot(hit[0] * MathUtil.M_TO_IN - tgt.x(), hit[1] * MathUtil.M_TO_IN - tgt.y());
                        worstMiss = Math.max(worstMiss, miss);
                        if (miss <= 8.6) hits++;
                        fsim.ballContact(t, 2.5, 0.04);
                        ball[0] = -1;
                    }
                }
                fsim.step(t, hstep);
                tsim.step(hstep);
                x += vx * hstep; y += vy * hstep; hdg += w * hstep;
                t += hstep;
            }
            hoodTruth.update(t);
        }
        check(moving ? "S1 moving: all fired balls land in the opening" : "S1 stationary: all fired balls land in the opening",
                fired >= 3 && hits == fired, String.format("%d/%d hits, worst miss %.1f in, first shot at %.2f s", hits, fired, worstMiss, firstShot));
        StringBuilder sb = new StringBuilder("     readiness time share: ");
        int total = 0;
        for (int v : reasons.values()) total += v;
        for (java.util.Map.Entry<ShooterCore.Readiness, Integer> e : reasons.entrySet()) {
            sb.append(String.format("%s %.0f%%  ", e.getKey(), 100.0 * e.getValue() / total));
        }
        System.out.println(sb);
    }

    // ------------------------------------------------------------------------------------------ alloc/timing
    static void allocationAndTimingTest() {
        System.out.println("\n== Loop cost ==");
        Ballistics b = new Ballistics(ballParams());
        ShotTable table = buildTable(b);
        ShooterCore core = new ShooterCore(new ShooterCore.Config(), new FlywheelController(fwConfig()),
                new TurretController(turretConfig()), new HoodModel(hoodConfig()), new AimSolver(new AimSolver.Config()),
                new HiveTarget(new HiveTarget.Config(), HiveTarget.Alliance.BLUE, HiveTarget.UpCell.FAR_SIDE), table);
        core.setMode(ShooterCore.Mode.TRACK);
        RobotState rs = new RobotState();
        double t = 0;
        for (int i = 0; i < 20000; i++) { // warm-up (JIT)
            t += 0.01;
            rs.set(-40 + i * 1e-3, -60, 0.3, 10, 0, 0.2, t);
            core.update(rs, t * 300, 0.1, 12.5, t);
        }
        java.lang.management.ThreadMXBean mx = java.lang.management.ManagementFactory.getThreadMXBean();
        long before = ((com.sun.management.ThreadMXBean) mx).getThreadAllocatedBytes(Thread.currentThread().getId());
        long n0 = System.nanoTime();
        int N = 20000;
        for (int i = 0; i < N; i++) {
            t += 0.01;
            rs.set(-40 + i * 1e-3, -60, 0.3, 10, 0, 0.2, t);
            core.update(rs, t * 300, 0.1, 12.5, t);
        }
        double usPer = (System.nanoTime() - n0) * 1e-3 / N;
        long after = ((com.sun.management.ThreadMXBean) mx).getThreadAllocatedBytes(Thread.currentThread().getId());
        check("S1 zero allocations per loop", (after - before) / (double) N < 1.0,
                String.format("%.2f bytes/loop", (after - before) / (double) N));
        check("S1 core compute per loop (PC; expect ~10–20× on Control Hub)", usPer < 30,
                String.format("%.2f µs/loop", usPer));
    }
}
