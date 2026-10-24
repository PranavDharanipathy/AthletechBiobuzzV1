package org.athletech.sim.scenarios;

import org.athletech.sim.config.Catalog;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.MatchDriver;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.undeterminables.ChargeSim;
import org.athletech.sim.undeterminables.Disconnects;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.undeterminables.Electrostatics;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Static electricity and disconnects. Plays matches with the full drive (and flywheel) physics to
 * get the robot's motion, battery voltage, current and hits; runs the field's static charge
 * through each match ({@link ChargeSim}); then turns discharges, brownouts, current surges and
 * drops, impacts and motor noise into disconnects ({@link Disconnects}). Repeats across humidity
 * with and without a grounding strap, and plays a whole competition day on one field so the
 * field's charge carries over. Fills esd.*, electrical.* and disconnect.* derived entries.
 */
public final class EsdScenario {
    private EsdScenario() {
    }

    static final double[] RH = {10, 20, 30, 40, 50, 60, 70};
    /** Charge-sim repeats per physics match (the other robots and contacts are re-drawn): cheap, and smooths the curves. */
    static final int REPS = 2;

    /** Everything read from the profile. */
    static final class Setup {
        MecanumDrive.Params drive;
        Launcher.Params launcher;
        double voc, rInt, drop, other, spread, holdS, didtS;
        ChargeSim.Params charge;
        MatchDriver.Behaviour behaviour;
        List<Disconnects.Mode> modes;
        int matchesPerField;
        double cycleMin;
        double capEstimatePf;
        boolean capEstimated;
    }

    static Setup read(RobotParams rp) {
        Setup s = new Setup();
        s.drive = rp.drive();
        rp.battery(Double.NaN);
        s.other = rp.electronicsCurrent();
        RobotParams lprobe = new RobotParams(rp.db());
        Launcher.Params lpp = lprobe.launcher();
        s.launcher = lprobe.missing().isEmpty() && lpp != null ? rp.launcher() : null;

        ChargeSim.Params c = new ChargeSim.Params();
        c.rh = rp.req("env.relative_humidity_pct");
        double len = rp.req("robot.length_m"), wid = rp.req("robot.width_m");
        double clear = rp.req("esd.chassis_height_m"), area = rp.opt("esd.frame_area_fraction", 0.5);
        double tileT = rp.opt("field.tile_thickness_m", 0.016), eps = rp.opt("field.tile_permittivity", 1.4);
        s.capEstimatePf = Electrostatics.robotCapacitance(len, wid, clear, area, tileT, eps) * 1e12;
        double capPf = rp.opt("esd.robot_capacitance_pf", Double.NaN);
        s.capEstimated = Double.isNaN(capPf);
        c.cFrame = (s.capEstimated ? s.capEstimatePf : capPf) * 1e-12;
        c.cCouple = rp.req("esd.electronics_coupling_pf") * 1e-12;
        c.cElec = rp.opt("esd.electronics_capacitance_pf", 30) * 1e-12;
        c.strap = rp.req("esd.grounding_strap") >= 0.5;
        c.strapOhm = rp.opt("esd.grounding_strap_ohm", 470);
        c.isoOhm40 = rp.opt("esd.isolation_ohm_at_40rh", 1e12);
        c.wheelOhm40 = rp.req("esd.wheel_leak_ohm_at_40rh");
        c.leakDecades = rp.opt("esd.leak_decades_per_10rh", 1);
        c.genCPerM40 = rp.req("esd.drive_charge_nc_per_m_at_40rh") * 1e-9;
        c.slideFactor = rp.opt("esd.slide_charge_factor", 2);
        c.slipFactor = rp.opt("esd.slip_charge_factor", 3);
        c.tileRhoS40 = rp.opt("field.tile_surface_resistivity_ohm_at_40rh", Double.NaN);
        c.tileCArea = org.athletech.sim.constants.Physics.EPSILON_0 * eps / tileT;
        c.genScalePct = rp.opt("esd.charge_humidity_scale_pct", 15);
        c.antistatic = rp.opt("esd.antistatic_factor", 1);
        c.intakeC = rp.opt("esd.intake_charge_nc_per_ball", 0) * 1e-9;
        c.wallHitC = rp.opt("esd.wall_hit_charge_nc", 0) * 1e-9;
        c.coronaV = rp.opt("esd.corona_onset_kv", 15) * 1e3;
        c.coronaBleedS = rp.opt("esd.corona_bleed_s", 0.1);
        c.cPerim = rp.req("esd.perimeter_capacitance_pf") * 1e-12;
        c.perimOhm40 = rp.req("esd.perimeter_leak_ohm_at_40rh");
        c.tileDecay40 = rp.req("esd.tile_decay_s_at_40rh");
        c.metalFraction = rp.req("esd.metal_contact_fraction");
        double cm = rp.opt("esd.metal_element_capacitance_pf", Double.NaN);
        c.cMetal = Double.isNaN(cm) ? Double.NaN : cm * 1e-12;
        c.metalOhm40 = rp.opt("esd.metal_element_leak_ohm_at_40rh", 1e11);
        c.internalBreakdownV = rp.req("esd.internal_breakdown_v");
        c.coupling = rp.req("esd.coupling_fraction");
        c.sparkOhm = rp.opt("esd.spark_impedance_ohm", 100);
        c.otherSpread = rp.opt("esd.other_robot_spread", 0.5);
        c.otherStrapFrac = rp.opt("esd.other_robot_strap_fraction", 0.5);
        c.fieldSize = rp.opt("field.size_m", 3.6576);
        c.length = len;
        c.width = wid;
        c.track = rp.req("drive.track_width_m");
        c.base = rp.req("drive.wheelbase_m");
        c.inducedFrac = Electrostatics.inducedFraction(clear, tileT, eps) * area;
        c.fieldGrounded = rp.flag("event.field_grounded", false);
        c.autoS = rp.opt("match.auto_s", 30);
        c.transitionS = rp.opt("match.transition_s", 8);
        c.teleopS = rp.opt("match.teleop_s", 120);

        MatchDriver.Behaviour b = new MatchDriver.Behaviour();
        b.autoS = c.autoS;
        b.transitionS = c.transitionS;
        b.teleopS = c.teleopS;
        b.fieldSizeM = c.fieldSize;
        b.lengthM = len;
        b.widthM = wid;
        b.wallHitsPerMatch = rp.req("driver.wall_hits_per_match");
        b.wallPushS = rp.opt("driver.wall_push_s", 0.6);
        b.robotContactsPerMatch = rp.req("driver.robot_contacts_per_match");
        b.intakesPerMatch = rp.opt("driver.intakes_per_match", 0);
        b.metalContactsPerMatch = rp.opt("driver.metal_element_contacts_per_match", 0);
        b.wallHitSpeed = known(rp.dist("driver.wall_hit_speed_mps"), Distribution.normal("wall", 0.8, 0.3, "m/s"), rp, "driver.wall_hit_speed_mps", "0.8 +/- 0.3 m/s");
        b.robotContactSpeed = known(rp.dist("driver.robot_contact_speed_mps"), Distribution.normal("robot", 0.7, 0.3, "m/s"), rp, "driver.robot_contact_speed_mps", "0.7 +/- 0.3 m/s");
        b.contactTimeMs = rp.opt("robot.contact_time_ms", 12);
        b.restitution = rp.opt("robot.wall_restitution", 0.2);
        b.ballsPerBurst = Math.max(1, (int) Math.round(rp.opt("robot.max_pollen_held", 3)));
        c.wallHitsPerMatch = b.wallHitsPerMatch;
        c.robotContactsPerMatch = b.robotContactsPerMatch;
        c.intakesPerMatch = b.intakesPerMatch;
        c.metalContactsPerMatch = b.metalContactsPerMatch;
        c.wallHitSpeed = b.wallHitSpeed;
        c.otherMaxSpeed = s.drive == null ? 1.5 : 0.9 * s.drive.theoreticalTopSpeed();
        s.charge = c;
        s.behaviour = b;

        s.spread = rp.opt("disconnect.fragility_spread", 0.6);
        s.holdS = rp.opt("electrical.brownout_hold_ms", 2) / 1000;
        s.didtS = rp.opt("electrical.didt_window_ms", 4) / 1000;
        s.modes = new ArrayList<>();
        for (String[] m : Catalog.DISCONNECT_MODES) {
            String pre = "disconnect." + m[0] + ".";
            boolean stops = m[0].equals("control_hub_reboot") || m[0].equals("wifi_drop");
            double esd = rp.opt(pre + "esd_energy_50_mj", Double.NaN);
            double emi = rp.opt(pre + "emi_rate_per_min_at_40a", Double.NaN), rnd = rp.opt(pre + "random_rate_per_min", Double.NaN);
            Distribution outage = rp.dist(pre + "outage_s");
            s.modes.add(new Disconnects.Mode(m[0], stops, esd / 1000, rp.opt(pre + "brownout_v", Double.NaN), rp.opt(pre + "impact_g_50", Double.NaN),
                    rp.opt(pre + "surge_50_a_per_ms", Double.NaN), rp.opt(pre + "drop_50_a_per_ms", Double.NaN),
                    Double.isNaN(emi) ? 0 : emi / 60, Double.isNaN(rnd) ? 0 : rnd / 60, outage));
        }
        s.matchesPerField = (int) Math.round(rp.opt("event.matches_per_field", 40));
        s.cycleMin = rp.opt("event.match_cycle_min", 7);
        return s;
    }

    private static Distribution known(Distribution d, Distribution fallback, RobotParams rp, String key, String text) {
        if (d.isKnown()) return d;
        rp.assumptions().add(key + " not set: using " + text);
        return fallback;
    }

    /** The robot's matches with full physics, in parallel (battery somewhere between full and 0.8 V down). */
    static MatchDriver.Trace[] traces(Setup s, int n, long seed) {
        MatchDriver.Trace[] out = new MatchDriver.Trace[n];
        IntStream.range(0, n).parallel().forEach(i -> {
            Rng rng = new Rng(seed * 7919 + i);
            double voc = s.voc - 0.8 * rng.uniform();
            out[i] = MatchDriver.simulate(s.drive, new Battery(voc, s.rInt, s.drop), s.other, s.launcher, s.behaviour, rng);
        });
        return out;
    }

    /** Result of one match of this robot under one configuration. */
    static final class Outcome {
        ChargeSim.MatchResult charge;
        Disconnects.Result disc;
    }

    static Outcome play(Setup s, ChargeSim.Params cp, MatchDriver.Trace tr, Rng rng, boolean series, int warmMatches) {
        ChargeSim.Field field = new ChargeSim.Field(cp.fieldSize);
        double gap = Math.max(0, s.cycleMin * 60 - cp.matchS());
        for (int w = 0; w < warmMatches; w++) {
            ChargeSim.runMatch(cp, null, field, rng, false);
            field.idle(gap, cp);
        }
        return evaluate(s, cp, tr, field, rng, series);
    }

    static Outcome evaluate(Setup s, ChargeSim.Params cp, MatchDriver.Trace tr, ChargeSim.Field field, Rng rng, boolean series) {
        Outcome o = new Outcome();
        o.charge = ChargeSim.runMatch(cp, tr, field, rng, series);
        List<double[]> esd = new ArrayList<>(), impacts = new ArrayList<>();
        for (ChargeSim.Event e : o.charge.events) esd.add(new double[]{e.t, e.coupledJ});
        for (double[] h : tr.wallHits) impacts.add(new double[]{h[0], h[2]});
        for (double[] h : tr.robotContacts) impacts.add(new double[]{h[0], h[2]});
        Disconnects.Stress st = new Disconnects.Stress(cp.matchS(), cp.autoS + cp.teleopS, esd, impacts, tr.busV, tr.current,
                MatchDriver.PHYSICS_DT, s.holdS, s.didtS);
        o.disc = Disconnects.evaluate(s.modes, st, s.spread, rng);
        return o;
    }

    public static void run(SimContext ctx) throws Exception {
        RobotParams rp = new RobotParams(ctx.db);
        Setup s = read(rp);
        rp.requireAll();
        s.voc = ctx.db.value("battery.open_circuit_v");
        s.rInt = ctx.db.value("battery.internal_resistance_ohm");
        s.drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        if (s.capEstimated) rp.assumptions().add("esd.robot_capacitance_pf not set: estimated " + Reports.round(s.capEstimatePf) + " pF from the robot's size and ground clearance");
        if (s.launcher == null) rp.assumptions().add("Launcher inputs incomplete: the flywheel's current is left out of the electrical stress");
        rp.assumptions().add("Every robot starts a match at 0 V (people handle robots between matches); the other three robots charge like this one, "
                + "varied by esd.other_robot_spread");
        SimContext.Saver save = ctx.saver(rp, "sim esd");
        ChargeSim.Params cp = s.charge;
        int n = Math.max(6, Math.min(48, ctx.trials / 60));
        ctx.log.printf("ESD + disconnect sim  (%.0f%% RH, frame %.0f pF, grounding strap %s, %d simulated matches)%n",
                cp.rh, cp.cFrame * 1e12, cp.strap ? "yes" : "no", n);

        // ---- the robot's matches (physics), reused for every humidity / strap setting
        MatchDriver.Trace[] tr = traces(s, n, ctx.seed);
        cp.otherSlipMps = otherSlip(tr, cp);
        double[] minV = new double[n], peakI = new double[n], peakG = new double[n], surge = new double[n], dropR = new double[n], voc = new double[n];
        double maxBrown = Double.NaN;
        for (Disconnects.Mode m : s.modes) if (m.brownoutV > 0) maxBrown = Double.isNaN(maxBrown) ? m.brownoutV : Math.max(maxBrown, m.brownoutV);
        double brownouts = 0;
        for (int i = 0; i < n; i++) {
            minV[i] = tr[i].minBusV();
            peakI[i] = tr[i].peakCurrent();
            peakG[i] = tr[i].peakImpactG();
            voc[i] = tr[i].voc;
            surge[i] = maxOf(Disconnects.slopePeaks(tr[i].current, MatchDriver.PHYSICS_DT, s.didtS, 0.5, true));
            dropR[i] = maxOf(Disconnects.slopePeaks(tr[i].current, MatchDriver.PHYSICS_DT, s.didtS, 0.5, false));
            if (maxBrown > 0) brownouts += Disconnects.brownouts(tr[i].busV, MatchDriver.PHYSICS_DT, maxBrown, s.holdS).size();
        }
        brownouts /= n;
        ctx.log.printf("  electrical: battery dips to %.2f V (5th pct), peak %.0f A, surge %.1f A/ms, drop %.1f A/ms, hits up to %.0f g%n",
                Stats.percentile(minV, 5), Stats.percentile(peakI, 50), Stats.percentile(surge, 50), Stats.percentile(dropR, 50), Stats.percentile(peakG, 50));

        // ---- at the profile's humidity
        Outcome[] base = new Outcome[n];
        IntStream.range(0, n).parallel().forEach(i -> base[i] = play(s, cp, tr[i], new Rng(ctx.seed * 131 + i), true, 2));
        int nm = s.modes.size();
        double[] perMode = new double[nm], atLeast = new double[nm];
        double[][] byCause = new double[nm][Disconnects.CAUSES.length];
        double anyProb = 0, down = 0, partial = 0, arcs = 0;
        double[] peakKv = new double[n], discharges = new double[n];
        List<Double> energies = new ArrayList<>();
        double[] hazard = new double[(int) Math.ceil(cp.matchS() / Disconnects.BIN_S)];
        for (int i = 0; i < n; i++) {
            Outcome o = base[i];
            for (int m = 0; m < nm; m++) {
                perMode[m] += o.disc.expected[m] / n;
                atLeast[m] += o.disc.atLeastOne[m] / n;
                for (int c = 0; c < Disconnects.CAUSES.length; c++) byCause[m][c] += o.disc.byCause[m][c] / n;
            }
            for (int b = 0; b < hazard.length && b < o.disc.hazardPerBin.length; b++) hazard[b] += o.disc.hazardPerBin[b] / n;
            anyProb += o.disc.anyProb / n;
            down += o.disc.downtimeS / n;
            partial += o.disc.partialDowntimeS / n;
            arcs += (double) o.charge.internalArcs / n;
            peakKv[i] = o.charge.maxAbsKv;
            for (ChargeSim.Event e : o.charge.events) {
                if (e.kind != ChargeSim.K_INTERNAL && e.energyJ > 1e-6) {
                    discharges[i]++;
                    energies.add(e.energyJ * 1000);
                }
            }
        }
        double[] en = energies.stream().mapToDouble(Double::doubleValue).toArray();
        double[] ledger = new double[ChargeSim.Ledger.NAMES.length];
        double generated = 0, slipShare = 0, worstResidual = 0;
        for (int i = 0; i < n; i++) {
            ChargeSim.MatchResult cr = base[i].charge;
            for (int j = 0; j < ledger.length; j++) ledger[j] += cr.ledger.c[j] / n;
            generated += cr.ledger.generated() / n;
            if (cr.ledger.generated() > 0) slipShare += cr.ledger.c[ChargeSim.Ledger.SLIP] / cr.ledger.generated() / n;
            if (cr.globalTurnoverC > 0) worstResidual = Math.max(worstResidual, Math.abs(cr.globalResidualC) / cr.globalTurnoverC);
            worstResidual = Math.max(worstResidual, Math.abs(cr.ledger.residual()) / Math.max(1e-30, cr.globalTurnoverC));
        }
        List<double[]> chargeVsI = chargingVsCurrent(tr, cp);
        ctx.log.printf("  charge: %.2f uC generated per match, %.0f%% of it from wheel slip; bookkeeping error %.1g of the charge moved%n",
                generated * 1e6, 100 * slipShare, worstResidual);
        ctx.log.printf("  static: frame reaches %.1f kV (median peak), %.1f discharges per match, %.2f internal arcs per match%n",
                Stats.percentile(peakKv, 50), Stats.mean(discharges), arcs);
        ctx.log.printf("  disconnects: %.1f%% chance per match, %.1f s fully stopped + %.1f s partly down per match on average%n",
                100 * anyProb, down, partial);

        // frame voltage through a match (median |V| per 1 s)
        List<double[]> kvVsT = new ArrayList<>();
        int sPts = base[0].charge.series.size();
        for (int j = 0; j < sPts; j += 2) {
            double[] col = new double[n];
            for (int i = 0; i < n; i++) col[i] = Math.abs(base[i].charge.series.get(Math.min(j, base[i].charge.series.size() - 1))[1]);
            kvVsT.add(new double[]{base[0].charge.series.get(j)[0], Stats.percentile(col, 50), Stats.std(col), n});
        }
        List<double[]> rateVsT = new ArrayList<>();
        for (int b = 0; b < hazard.length; b++) rateVsT.add(new double[]{(b + 0.5) * Disconnects.BIN_S, hazard[b] / Disconnects.BIN_S * 60});

        // ---- humidity sweeps: as built, with and without a strap
        double[][] sweepProb = new double[3][RH.length], sweepKv = new double[3][RH.length], sweepDis = new double[3][RH.length];
        boolean[] straps = {cp.strap, true, false};
        IntStream.range(0, 3 * RH.length).parallel().forEach(k -> {
            int v = k / RH.length, j = k % RH.length;
            if (v > 0 && straps[v] == cp.strap) return; // same as "as built": copied below
            ChargeSim.Params q = cp.withRh(RH[j]).withStrap(straps[v]);
            double prob = 0, dis = 0;
            double[] pk = new double[n * REPS];
            for (int i = 0; i < n * REPS; i++) {
                Outcome o = play(s, q, tr[i % n], new Rng(ctx.seed * 977 + k * 1009 + i), false, 2);
                prob += o.disc.anyProb / (n * REPS);
                pk[i] = o.charge.maxAbsKv;
                for (ChargeSim.Event e : o.charge.events) if (e.kind != ChargeSim.K_INTERNAL && e.energyJ > 1e-6) dis += 1.0 / (n * REPS);
            }
            sweepProb[v][j] = prob;
            sweepKv[v][j] = Stats.percentile(pk, 50);
            sweepDis[v][j] = dis;
        });
        for (int v = 1; v < 3; v++) {
            if (straps[v] == cp.strap) {
                sweepProb[v] = sweepProb[0].clone();
                sweepKv[v] = sweepKv[0].clone();
                sweepDis[v] = sweepDis[0].clone();
            }
        }
        List<double[]> probVsRh = new ArrayList<>(), probStrap = new ArrayList<>(), probNoStrap = new ArrayList<>(), kvVsRh = new ArrayList<>(), disVsRh = new ArrayList<>();
        for (int j = 0; j < RH.length; j++) {
            probVsRh.add(new double[]{RH[j], sweepProb[0][j]});
            probStrap.add(new double[]{RH[j], sweepProb[1][j]});
            probNoStrap.add(new double[]{RH[j], sweepProb[2][j]});
            kvVsRh.add(new double[]{RH[j], sweepKv[0][j]});
            disVsRh.add(new double[]{RH[j], sweepDis[0][j]});
        }

        // ---- a competition day on one field
        int days = Math.max(2, ctx.trials / 250);
        int mpf = Math.max(6, s.matchesPerField);
        double gap = Math.max(0, s.cycleMin * 60 - cp.matchS());
        double[] perimKv = new double[mpf], tileUc = new double[mpf];
        List<double[]> dayProb = new ArrayList<>();
        double[][] dayProbAcc = new double[mpf][2];
        double[] endKv = new double[days];
        for (int dIdx = 0; dIdx < days; dIdx++) {
            Rng rng = new Rng(ctx.seed * 4241 + dIdx);
            ChargeSim.Field field = new ChargeSim.Field(cp.fieldSize);
            for (int m = 0; m < mpf; m++) {
                boolean ourMatch = m % 6 == 2;
                if (ourMatch) {
                    Outcome o = evaluate(s, cp, tr[(dIdx * 7 + m) % n], field, rng, false);
                    dayProbAcc[m][0] += o.disc.anyProb;
                    dayProbAcc[m][1]++;
                } else {
                    ChargeSim.runMatch(cp, null, field, rng, false);
                }
                perimKv[m] += field.perimeterV(cp) / 1000 / days;
                tileUc[m] += field.tiles.total() * 1e6 / days;
                if (m == mpf - 1) endKv[dIdx] = field.perimeterV(cp) / 1000;
                field.idle(gap, cp);
            }
        }
        List<double[]> perimVsT = new ArrayList<>(), tileVsT = new ArrayList<>();
        for (int m = 0; m < mpf; m++) {
            double hours = (m * s.cycleMin * 60 + cp.matchS()) / 3600;
            perimVsT.add(new double[]{hours, perimKv[m]});
            tileVsT.add(new double[]{hours, tileUc[m]});
            if (dayProbAcc[m][1] > 0) dayProb.add(new double[]{m + 1, dayProbAcc[m][0] / dayProbAcc[m][1], Double.NaN, dayProbAcc[m][1]});
        }

        // ---- electrical: lowest voltage vs resting voltage
        List<double[]> minVsVoc = new ArrayList<>();
        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> Double.compare(voc[a], voc[b]));
        int bins = Math.max(2, Math.min(5, n / 3));
        for (int b = 0; b < bins; b++) {
            int lo = b * n / bins, hi = (b + 1) * n / bins;
            double sx = 0, sy = 0;
            for (int i = lo; i < hi; i++) {
                sx += voc[order[i]];
                sy += minV[order[i]];
            }
            minVsVoc.add(new double[]{sx / (hi - lo), sy / (hi - lo), Double.NaN, hi - lo});
        }

        // ---- save
        save.param("esd.robot_capacitance_estimate_pf", s.capEstimatePf);
        save.param("esd.charge_generated_uc_per_match", generated * 1e6);
        save.param("esd.slip_charge_share", slipShare);
        save.curve("esd.charging_rate_vs_battery_current", chargeVsI);
        save.param("esd.peak_robot_kv", Stats.percentile(peakKv, 50));
        save.param("esd.discharges_per_match", Stats.mean(discharges));
        save.param("esd.discharge_energy_p95_mj", en.length > 0 ? Stats.percentile(en, 95) : Double.NaN);
        save.param("esd.internal_arcs_per_match", arcs);
        save.param("esd.perimeter_kv_end_of_day", Stats.percentile(endKv, 50));
        save.param("electrical.min_bus_v", Stats.percentile(minV, 5));
        save.param("electrical.peak_current_a", Stats.percentile(peakI, 50));
        save.param("electrical.peak_surge_a_per_ms", Stats.percentile(surge, 50));
        save.param("electrical.peak_drop_a_per_ms", Stats.percentile(dropR, 50));
        save.param("electrical.peak_impact_g", Stats.percentile(peakG, 50));
        save.param("electrical.brownouts_per_match", Double.isNaN(maxBrown) ? Double.NaN : brownouts);
        for (int m = 0; m < nm; m++) save.param("disconnect." + s.modes.get(m).id + ".per_match", perMode[m]);
        save.param("disconnect.prob_per_match", anyProb);
        save.param("disconnect.downtime_s_per_match", down);
        save.param("disconnect.partial_downtime_s_per_match", partial);
        if (en.length > 2) {
            save.dist("esd.discharge_energy_mj", "lognormal", Stats.mean(en), Stats.std(en));
        } else {
            save.dist("esd.discharge_energy_mj", "lognormal", Double.NaN, Double.NaN);
        }
        save.curve("esd.robot_kv_vs_match_time", kvVsT);
        save.curve("esd.peak_robot_kv_vs_humidity", kvVsRh);
        save.curve("esd.discharges_per_match_vs_humidity", disVsRh);
        save.curve("esd.perimeter_kv_vs_event_time", perimVsT);
        save.curve("esd.tile_charge_uc_vs_event_time", tileVsT);
        save.curve("electrical.min_bus_v_vs_battery_v", minVsVoc);
        save.curve("disconnect.prob_per_match_vs_humidity", probVsRh);
        save.curve("disconnect.prob_per_match_vs_humidity_with_strap", probStrap);
        save.curve("disconnect.prob_per_match_vs_humidity_no_strap", probNoStrap);
        save.curve("disconnect.rate_vs_match_time", rateVsT);
        save.curve("disconnect.prob_vs_event_match", dayProb);

        // ---- report
        HtmlReport r = new HtmlReport("Static electricity and disconnects");
        Reports.provenance(r, rp, ctx);
        r.paragraph("Each simulated match plays the drive (and flywheel) physics for the robot's motion, battery voltage, current and hits, "
                + "then runs the static charge on the robot, three other robots, the field perimeter, metal structures and the tiles, "
                + "and turns every discharge, brownout, current surge or drop, impact and stretch of motor noise into disconnect chances. "
                + n + " matches per setting.");
        r.heading("Summary at " + Reports.round(cp.rh) + "% humidity");
        r.table(Arrays.asList("quantity", "value", "unit"), Arrays.asList(
                Reports.row("chance of at least one disconnect", 100 * anyProb, "% per match"),
                Reports.row("robot fully stopped (Control Hub reboot, Wi-Fi)", down, "s per match"),
                Reports.row("part of the robot down (Expansion Hub, I2C, camera)", partial, "s per match"),
                Reports.row("frame capacitance" + (s.capEstimated ? " (estimated)" : ""), cp.cFrame * 1e12, "pF"),
                Reports.row("highest frame voltage (median match)", Stats.percentile(peakKv, 50), "kV"),
                Reports.row("discharges per match (> 1 uJ)", Stats.mean(discharges), "count"),
                Reports.row("discharge energy, 95th percentile", en.length > 0 ? Stats.percentile(en, 95) : Double.NaN, "mJ"),
                Reports.row("arcs inside the robot", arcs, "per match"),
                Reports.row("field perimeter after a full day (median)", Stats.percentile(endKv, 50), "kV"),
                Reports.row("lowest battery voltage (5th percentile match)", Stats.percentile(minV, 5), "V"),
                Reports.row("peak current (median match)", Stats.percentile(peakI, 50), "A"),
                Reports.row("fastest current rise / drop (median match)", Reports.round(Stats.percentile(surge, 50)) + " / " + Reports.round(Stats.percentile(dropR, 50)), "A/ms"),
                Reports.row("hardest hit (median match)", Stats.percentile(peakG, 50), "g"),
                Reports.row("brownouts below " + (Double.isNaN(maxBrown) ? "-" : Reports.round(maxBrown) + " V"), brownouts, "per match")));
        List<String> header = new ArrayList<>(Arrays.asList("disconnect", "per match", "chance per match %"));
        for (String c : Disconnects.CAUSES) header.add(c);
        List<List<String>> rows = new ArrayList<>();
        for (int m = 0; m < nm; m++) {
            List<String> row = new ArrayList<>(Arrays.asList(s.modes.get(m).id.replace('_', ' '), small(perMode[m]), small(100 * atLeast[m])));
            for (int c = 0; c < Disconnects.CAUSES.length; c++) row.add(small(byCause[m][c]));
            rows.add(row);
        }
        r.table(header, rows);
        r.heading("Where this robot's charge comes from and goes");
        List<List<String>> ledRows = new ArrayList<>();
        for (int j = 0; j < ledger.length; j++) ledRows.add(Reports.row(ChargeSim.Ledger.NAMES[j], ledger[j] * 1e9, generated > 0 ? 100 * ledger[j] / generated : Double.NaN));
        r.table(Arrays.asList("source (+) or sink (-)", "nC per match (average)", "% of the charge generated"), ledRows);
        r.paragraph("Every coulomb is tracked: the robot's charge at the end of a match equals the sum of these rows, and the whole field "
                + "(four robots, perimeter, metal structures, tiles) balances against contact charging in and leakage out. Largest "
                + "bookkeeping error over these matches: " + Reports.round(worstResidual) + " of the charge moved (rounding).");
        r.heading("Charge through a match");
        r.chart(new SvgChart("Frame voltage through a match (median of " + n + ")", "match time (s)", "kV").zeroBased().add(Reports.series("|V| frame", kvVsT, true)));
        ChargeSim.MatchResult ex = base[0].charge;
        List<double[]> exF = new ArrayList<>(), exD = new ArrayList<>();
        for (double[] q : ex.series) {
            exF.add(new double[]{q[0], q[1]});
            exD.add(new double[]{q[0], q[2]});
        }
        r.chart(new SvgChart("One match: frame and electronics", "match time (s)", "kV").add(Reports.series("frame", exF)).add(Reports.series("electronics minus frame", exD)));
        if (!chargeVsI.isEmpty()) {
            r.chart(new SvgChart("Charging rate vs battery current", "total battery current (A)", "nC/s").zeroBased().add(Reports.series("charging", chargeVsI, true)));
        }
        r.chart(new SvgChart("Disconnects through a match", "match time (s)", "per minute").zeroBased().add(Reports.series("all kinds", rateVsT)));
        List<List<String>> evRows = new ArrayList<>();
        for (ChargeSim.Event e : ex.events) {
            if (evRows.size() >= 25) break;
            if (e.energyJ < 1e-7) continue;
            evRows.add(Reports.row(e.t, ChargeSim.KIND_NAMES[e.kind], e.dv / 1000, e.energyJ * 1000, e.coupledJ * 1000, e.peakA));
        }
        if (!evRows.isEmpty()) {
            r.paragraph("Discharges in that match (first 25 over 0.1 uJ):");
            r.table(Arrays.asList("time s", "discharge", "dV kV", "energy mJ", "into electronics mJ", "peak A"), evRows);
        }
        r.heading("Humidity and the grounding strap");
        r.chart(new SvgChart("Chance of a disconnect per match", "relative humidity (%)", "probability").yRange(0, 1)
                .add(Reports.series("as built", probVsRh)).add(Reports.series("with strap", probStrap)).add(Reports.series("no strap", probNoStrap)));
        r.chart(new SvgChart("Highest frame voltage per match", "relative humidity (%)", "kV").zeroBased().add(Reports.series("median", kvVsRh)));
        r.chart(new SvgChart("Discharges per match", "relative humidity (%)", "count").zeroBased().add(Reports.series("discharges", disVsRh)));
        r.heading("A competition day on one field (" + mpf + " matches, every " + Reports.round(s.cycleMin) + " min)");
        r.chart(new SvgChart("Field perimeter voltage", "hours since the first match", "kV").add(Reports.series("perimeter", perimVsT)));
        r.chart(new SvgChart("Net charge on the tiles", "hours since the first match", "uC").add(Reports.series("tiles", tileVsT)));
        r.chart(new SvgChart("Chance of a disconnect in this robot's matches", "match number that day", "probability").yRange(0, 1).add(Reports.series("this robot", dayProb)));
        r.heading("Electrical stress");
        MatchDriver.Trace t0 = tr[0];
        List<double[]> vT = new ArrayList<>(), iT = new ArrayList<>();
        for (int i = 0; i + 100 <= t0.busV.length; i += 100) {
            double lo = Double.POSITIVE_INFINITY, hi = 0;
            for (int j = i; j < i + 100; j++) {
                lo = Math.min(lo, t0.busV[j]);
                hi = Math.max(hi, t0.current[j]);
            }
            vT.add(new double[]{i * MatchDriver.PHYSICS_DT, lo});
            iT.add(new double[]{i * MatchDriver.PHYSICS_DT, hi});
        }
        r.chart(new SvgChart("One match: battery voltage (lowest per 0.1 s)", "match time (s)", "V").add(Reports.series("battery", vT)));
        r.chart(new SvgChart("One match: total current (highest per 0.1 s)", "match time (s)", "A").zeroBased().add(Reports.series("current", iT)));
        r.chart(new SvgChart("Lowest voltage in a match vs resting voltage", "resting voltage (V)", "V").add(Reports.series("lowest", minVsVoc)));
        r.bullets(Arrays.asList(
                "Leakage scales 10x per " + Reports.round(10 / Math.max(1e-6, cp.leakDecades)) + "% humidity; charging falls by e per " + Reports.round(cp.genScalePct) + "% humidity (both relative to 40%).",
                "Charging at this humidity: " + Reports.round(cp.genPerM() * 1e9) + " nC per meter rolled (x" + Reports.round(cp.slideFactor) + " when sliding, x"
                        + Reports.round(cp.slipFactor) + " per meter of wheel slip); "
                        + "frame leak time constant " + Reports.round(cp.cFrame / cp.gWheel()) + " s; tile charge lifetime " + Reports.round(cp.tileTau()) + " s"
                        + (cp.tileDiffusion() > 0 ? "; tile charge creeps sideways about " + Reports.round(Math.sqrt(cp.tileDiffusion() * cp.tileTau())) + " m in that time" : "") + ".",
                "A discharge of energy E reaching the electronics fails a device with probability Phi(ln(E / E50) / " + Reports.round(s.spread) + "); "
                        + "a device under its reset voltage for " + Reports.round(s.holdS * 1000) + " ms resets."));
        r.write(ctx.outDir.resolve("esd/report.html"));
        ctx.log.println("  report: " + ctx.outDir.resolve("esd/report.html"));
        save.report();
    }

    /**
     * {@code predict disconnect}: chance of a disconnect per match at a given humidity
     * (--humidity RH, default the profile's) with or without a strap (--strap 0|1).
     */
    public static int predict(org.athletech.sim.config.ProfileDb db, org.athletech.sim.utility.Args a, java.io.PrintStream out) throws Exception {
        RobotParams rp = new RobotParams(db);
        Setup s = read(rp);
        rp.requireAll();
        s.voc = db.value("battery.open_circuit_v");
        s.rInt = db.value("battery.internal_resistance_ohm");
        s.drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        ChargeSim.Params cp = s.charge.withRh(a.num("humidity", s.charge.rh));
        if (a.has("strap")) cp = cp.withStrap(a.num("strap", 1) >= 0.5);
        final ChargeSim.Params q = cp;
        int n = Math.max(4, Math.min(32, a.integer("trials", 1000) / 120));
        long seed = (long) a.num("seed", 1);
        MatchDriver.Trace[] tr = traces(s, n, seed);
        q.otherSlipMps = otherSlip(tr, q);
        Outcome[] o = new Outcome[n * REPS];
        IntStream.range(0, n * REPS).parallel().forEach(i -> o[i] = play(s, q, tr[i % n], new Rng(seed * 53 + i), false, 2));
        double prob = 0, down = 0, partial = 0, kv = 0;
        double[] perMode = new double[s.modes.size()];
        for (Outcome x : o) {
            prob += x.disc.anyProb / o.length;
            down += x.disc.downtimeS / o.length;
            partial += x.disc.partialDowntimeS / o.length;
            kv += x.charge.maxAbsKv / o.length;
            for (int m = 0; m < perMode.length; m++) perMode[m] += x.disc.expected[m] / o.length;
        }
        out.printf(java.util.Locale.ROOT, "At %.0f%% humidity, %s grounding strap (%d simulated matches):%n", q.rh, q.strap ? "with a" : "without a", o.length);
        out.printf(java.util.Locale.ROOT, "  %.1f%% chance of at least one disconnect per match; %.1f s fully stopped + %.1f s partly down on average%n",
                100 * prob, down, partial);
        out.printf(java.util.Locale.ROOT, "  frame reaches %.1f kV on average%n", kv);
        for (int m = 0; m < perMode.length; m++) {
            if (perMode[m] > 1e-4) out.printf(java.util.Locale.ROOT, "  %-20s %.4f per match%n", s.modes.get(m).id, perMode[m]);
        }
        if (!rp.placeholderInputs().isEmpty()) out.println("  uses PLACEHOLDER inputs (" + rp.placeholderInputs().size() + " of them)");
        return 0;
    }

    /** Other robots slip like this one on average: its wheel-slip meters per second of enabled time. */
    static double otherSlip(MatchDriver.Trace[] tr, ChargeSim.Params cp) {
        double m = 0;
        for (MatchDriver.Trace t : tr) m += t.slipM / tr.length;
        return m / Math.max(1, cp.autoS + cp.teleopS);
    }

    /**
     * Charging rate (nC/s) against total battery current (A), binned: high current means hard
     * pushing or launching, where wheels slip and charge the frame fastest. {x A, mean nC/s, std, n}.
     */
    static List<double[]> chargingVsCurrent(MatchDriver.Trace[] tr, ChargeSim.Params cp) {
        double genPerM = cp.genPerM(), k = (cp.track + cp.base) / 2, bin = 2.0;
        java.util.TreeMap<Integer, double[]> bins = new java.util.TreeMap<>();
        double enabledEnd = cp.autoS, teleStart = cp.autoS + cp.transitionS;
        for (MatchDriver.Trace t : tr) {
            for (int i = 0; i < t.x.length; i++) {
                double time = i * MatchDriver.REC_DT;
                if (time >= enabledEnd && time < teleStart) continue;
                double rate = genPerM * 1e9 * (Math.abs(t.vf[i]) + cp.slideFactor * (Math.abs(t.vl[i]) + Math.abs(t.w[i]) * k) + cp.slipFactor * t.slip[i]);
                double[] b = bins.computeIfAbsent((int) Math.floor(t.currentAt(i) / bin), x -> new double[3]);
                b[0] += rate;
                b[1] += rate * rate;
                b[2]++;
            }
        }
        List<double[]> out = new ArrayList<>();
        for (java.util.Map.Entry<Integer, double[]> e : bins.entrySet()) {
            double[] b = e.getValue();
            if (b[2] < 20) continue;
            double mean = b[0] / b[2];
            out.add(new double[]{(e.getKey() + 0.5) * bin, mean, Math.sqrt(Math.max(0, b[1] / b[2] - mean * mean)), b[2]});
        }
        return out;
    }

    /** Readable small numbers for the cause table: blank for 0, "<0.0001" for tiny. */
    static String small(double v) {
        if (!(v > 0)) return "";
        if (v < 1e-4) return "<0.0001";
        return org.athletech.sim.utility.Csv.fmt(Reports.round(v));
    }

    static double maxOf(List<double[]> peaks) {
        double m = 0;
        for (double[] p : peaks) m = Math.max(m, p[1]);
        return m;
    }
}
