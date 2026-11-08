package org.athletech.sim.undeterminables;

import org.athletech.sim.robot.MatchDriver;
import org.athletech.sim.utility.Rng;

import java.util.ArrayList;
import java.util.List;

public final class ChargeSim {
    private ChargeSim() {
    }

    public static final double DT = 0.01;
    public static final int GRID = 24;

    public static final int K_PERIMETER = 0, K_ROBOT = 1, K_METAL = 2, K_INTERNAL = 3;
    public static final String[] KIND_NAMES = {"frame to perimeter metal", "frame to another robot", "frame to metal structure", "electronics to frame (inside)"};

    public static final class Params implements Cloneable {
        public double rh = 30;
        public double cFrame = 70e-12, cElec = 30e-12, cCouple = 300e-12;
        public boolean strap = true;
        public double strapOhm = 470, isoOhm40 = 1e12, wheelOhm40 = 1e11, leakDecades = 1;
        public double genCPerM40 = 8e-9, slideFactor = 2, slipFactor = 3, genScalePct = 15, antistatic = 1;
        public double otherSlipMps = 0.02;
        public double tileRhoS40 = Double.NaN, tileCArea = 7.7e-10;
        public double intakeC = 2e-9, wallHitC = 5e-9, coronaV = 15e3, coronaBleedS = 0.1;
        public double cPerim = 400e-12, perimOhm40 = 2e10, tileDecay40 = 300, metalFraction = 0.3;
        public double cMetal = Double.NaN, metalOhm40 = 1e11;
        public double internalBreakdownV = 1500, coupling = 0.02, sparkOhm = 100;
        public double otherSpread = 0.5, otherStrapFrac = 0.5;
        public double fieldSize = 3.6576, length = 0.43, width = 0.43, track = 0.36, base = 0.30, inducedFrac = 0.15;
        public double wallHitsPerMatch = 6, robotContactsPerMatch = 4, intakesPerMatch = 20, metalContactsPerMatch = 2;
        public Distribution wallHitSpeed = Distribution.normal("wall", 0.8, 0.3, "m/s");
        public double otherMaxSpeed = 1.6;
        public boolean fieldGrounded;
        public double autoS = 30, transitionS = 8, teleopS = 120;

        public Params copy() {

            try {
                return (Params) clone();
            } catch (CloneNotSupportedException e) {
                throw new IllegalStateException(e);
            }
        }

        public Params withRh(double v) {

            Params p = copy();
            p.rh = v;
            return p;
        }

        public Params withStrap(boolean v) {

            Params p = copy();
            p.strap = v;
            return p;
        }

        public double matchS() { return autoS + transitionS + teleopS; }

        boolean enabled(double t) { return t < autoS || t >= autoS + transitionS; }

        public double genPerM() { return genCPerM40 * Electrostatics.generationFactor(rh, genScalePct) * antistatic; }

        double resFactor() { return Electrostatics.resistanceFactor(rh, leakDecades); }

        public double gWheel() { return 1 / (wheelOhm40 * resFactor()); }

        double gIso() { return 1 / (isoOhm40 * resFactor()); }

        double gPerimeter() { return fieldGrounded ? 1e-6 : 1 / (perimOhm40 * resFactor()); }

        double gMetal() { return 1 / (metalOhm40 * resFactor()); }

        public double tileTau() { return tileDecay40 * resFactor() * antistatic; }

        public double tileDiffusion() {

            if (!(tileRhoS40 > 0) || !(tileCArea > 0)) return 0;
            return 1 / (tileRhoS40 * resFactor() * antistatic * tileCArea);
        }
    }

    public static final class Field {
        public double qPerimeter, qMetal;
        public final TileChargeGrid tiles;

        public Field(double fieldSizeM) {
            tiles = new TileChargeGrid(fieldSizeM, GRID);
        }

        public void idle(double seconds, Params p) {

            qPerimeter *= Math.exp(-seconds * p.gPerimeter() / p.cPerim);
            if (p.cMetal > 0) qMetal *= Math.exp(-seconds * p.gMetal() / p.cMetal);
            tiles.decay(Math.exp(-seconds / p.tileTau()));
            tiles.diffuse(p.tileDiffusion(), seconds, 20000);
        }

        public double total() { return qPerimeter + qMetal + tiles.total(); }

        public double perimeterV(Params p) { return qPerimeter / p.cPerim; }
    }

    public static final class Event {
        public final double t, dv, energyJ, coupledJ, peakA;
        public final int kind;

        Event(double t, int kind, double dv, double energyJ, double coupledJ, double peakA) {

            this.t = t;
            this.kind = kind;
            this.dv = dv;
            this.energyJ = energyJ;
            this.coupledJ = coupledJ;
            this.peakA = peakA;
        }
    }

    public static final class Ledger {
        public static final String[] NAMES = {"rolling over the tiles", "sliding (strafe, turn)", "wheel slip (pushing, hard starts)",
                "intaking game elements", "hitting plastic walls", "perimeter metal", "other robots", "metal structures",
                "leak to the floor (wheels)", "leak into the air (corona)"};
        public static final int ROLL = 0, SLIDE = 1, SLIP = 2, INTAKE = 3, WALL = 4, PERIMETER = 5, ROBOTS = 6, METAL = 7, FLOOR = 8, AIR = 9;
        public final double[] c = new double[NAMES.length];
        public double qStart, qEnd;

        public double generated() {
            return c[ROLL] + c[SLIDE] + c[SLIP] + c[INTAKE] + c[WALL];
        }

        public double residual() {

            double sum = 0;
            for (double v : c) sum += v;
            return (qEnd - qStart) - sum;
        }
    }

    public static final class MatchResult {
        public final List<Event> events = new ArrayList<>();
        public final List<double[]> series = new ArrayList<>();
        public double maxAbsKv, perimeterKvEnd, tileUcEnd;
        public int internalArcs;
        public final Ledger ledger = new Ledger();
        public double globalResidualC;
        public double globalTurnoverC;
    }

    private static final class Bot {
        final RobotCharge charge;
        final double genMult;
        final boolean strap;
        double x, y, h, vf, vl, w, slip;
        double speed, tx, ty, th, pause, vmax;

        Bot(Params p, double genMult, boolean strap) {

            this.charge = new RobotCharge(p.cFrame, p.cElec, p.cCouple);
            this.genMult = genMult;
            this.strap = strap;
        }

        void target(Rng rng, double lim) {

            tx = rng.uniform(-lim, lim);
            ty = rng.uniform(-lim, lim);
            th = rng.uniform(-Math.PI, Math.PI);
        }

        void drive(double dt, Rng rng, double lim, boolean on) {

            vf = vl = w = 0;
            if (!on) {
                speed = 0;
                return;
            }
            if (pause > 0) {
                pause -= dt;
                speed = 0;
                return;
            }
            double dx = tx - x, dy = ty - y, dist = Math.hypot(dx, dy);
            if (dist < 0.05) {
                pause = rng.uniform(0.4, 2.5);
                target(rng, lim);
                return;
            }
            double accel = 3.0;
            double vt = Math.min(vmax, Math.sqrt(2 * accel * dist));
            speed = speed < vt ? Math.min(vt, speed + accel * dt) : vt;
            double dir = Math.atan2(dy, dx);
            x += Math.cos(dir) * speed * dt;
            y += Math.sin(dir) * speed * dt;
            double dh = Math.IEEEremainder(th - h, 2 * Math.PI);
            w = Math.max(-3, Math.min(3, dh / dt));
            h += w * dt;
            vf = speed * Math.cos(dir - h);
            vl = speed * Math.sin(dir - h);
        }
    }

    public static MatchResult runMatch(Params p, MatchDriver.Trace ours, Field field, Rng rng, boolean recordSeries) {

        MatchResult res = new MatchResult();
        int nBots = 4;
        Bot[] bots = new Bot[nBots];
        double lim = p.fieldSize / 2 - 0.3;
        for (int i = 0; i < nBots; i++) {
            boolean isOurs = i == 0 && ours != null;
            double mult = isOurs ? 1 : Math.exp(p.otherSpread * rng.gaussian() - p.otherSpread * p.otherSpread / 2);
            boolean strap = isOurs ? p.strap : rng.uniform() < p.otherStrapFrac;
            bots[i] = new Bot(p, mult, strap);
            bots[i].x = rng.uniform(-lim, lim);
            bots[i].y = rng.uniform(-lim, lim);
            bots[i].vmax = p.otherMaxSpeed * rng.uniform(0.7, 1.1);
            bots[i].target(rng, lim);
        }
        double matchS = p.matchS();
        double active = p.autoS + p.teleopS;
        double genPerM = p.genPerM(), genFactor = Electrostatics.generationFactor(p.rh, p.genScalePct);
        double gW = p.gWheel(), gIso = p.gIso();
        double k = (p.track + p.base) / 2;
        double tileFactor = Math.exp(-10 * DT / p.tileTau());
        double tileD = p.tileDiffusion();
        double perimFactor = Math.exp(-DT * p.gPerimeter() / p.cPerim);
        double metalFactor = p.cMetal > 0 ? Math.exp(-DT * p.gMetal() / p.cMetal) : 1;
        double pWall = p.wallHitsPerMatch / active * DT, pIntake = p.intakesPerMatch / active * DT;
        double pContact = p.robotContactsPerMatch / active * DT, pMetal = p.metalContactsPerMatch / active * DT;
        int wi = 0, ci = 0, ii = 0, mi = 0;
        int steps = (int) Math.round(matchS / DT);
        double[] qInd = new double[nBots];
        double nextSeries = 0;
        Ledger led = res.ledger;
        double globalStart = field.total(), external = 0, drained = 0, turnover = 0;
        for (int s = 0; s < steps; s++) {
            double t = s * DT;
            boolean on = p.enabled(t);
            for (int i = 0; i < nBots; i++) {
                Bot b = bots[i];
                if (i == 0 && ours != null) {
                    int r = Math.min(ours.x.length - 1, (int) Math.round(t / MatchDriver.REC_DT));
                    b.x = ours.x[r];
                    b.y = ours.y[r];
                    b.h = ours.h[r];
                    b.vf = ours.vf[r];
                    b.vl = ours.vl[r];
                    b.w = ours.w[r];
                    b.slip = ours.slip[r];
                }
                else {
                    b.drive(DT, rng, lim, on);
                }
            }
            for (int i = 0; i < nBots; i++) {
                Bot b = bots[i];
                boolean isOurs = i == 0 && ours != null;
                double roll = Math.abs(b.vf), slide = p.slideFactor * (Math.abs(b.vl) + Math.abs(b.w) * k);
                double slip = p.slipFactor * (isOurs ? b.slip : (on ? p.otherSlipMps : 0));
                double rate = roll + slide + slip;
                if (rate <= 0) continue;
                double dq = genPerM * b.genMult * rate * DT;
                b.charge.addFrame(dq);
                turnover += Math.abs(dq);
                if (isOurs) {
                    led.c[Ledger.ROLL] += dq * roll / rate;
                    led.c[Ledger.SLIDE] += dq * slide / rate;
                    led.c[Ledger.SLIP] += dq * slip / rate;
                }
                double c = Math.cos(b.h), sn = Math.sin(b.h);
                for (int wx = -1; wx <= 1; wx += 2) {
                    for (int wy = -1; wy <= 1; wy += 2) {
                        double ox = wx * p.base / 2, oy = wy * p.track / 2;
                        field.tiles.deposit(b.x + c * ox - sn * oy, b.y + sn * ox + c * oy, -dq / 4);
                    }
                }
            }
            for (int i = 0; i < nBots; i++) {
                Bot b = bots[i];
                qInd[i] = p.inducedFrac * field.tiles.underFootprint(b.x, b.y, b.h, p.length, p.width);
            }
            if (ours != null) {
                while (wi < ours.wallHits.size() && ours.wallHits.get(wi)[0] < t + DT) {
                    double[] e = ours.wallHits.get(wi++);
                    if (rng.uniform() < p.metalFraction) {
                        double[] x = toPerimeter(bots[0], qInd[0], field, p);
                        led.c[Ledger.PERIMETER] -= x[2];
                        turnover += Math.abs(x[2]);
                        record(res, t, K_PERIMETER, x, p, bots[0], qInd[0]);
                    }
                    else {
                        double dq = p.wallHitC * genFactor * Math.min(3, e[1]);
                        bots[0].charge.addFrame(dq);
                        led.c[Ledger.WALL] += dq;
                        external += dq;
                        turnover += Math.abs(dq);
                    }
                }
                while (ci < ours.robotContacts.size() && ours.robotContacts.get(ci)[0] < t + DT) {
                    ci++;
                    int j = 1 + (int) (rng.uniform() * (nBots - 1));
                    double[] x = between(bots[0], qInd[0], bots[j], qInd[j]);
                    led.c[Ledger.ROBOTS] -= x[2];
                    turnover += Math.abs(x[2]);
                    record(res, t, K_ROBOT, x, p, bots[0], qInd[0]);
                    internal(bots[j], qInd[j], p);
                }
                while (ii < ours.intakes.size() && ours.intakes.get(ii) < t + DT) {
                    ii++;
                    double dq = p.intakeC * genFactor;
                    bots[0].charge.addFrame(dq);
                    led.c[Ledger.INTAKE] += dq;
                    external += dq;
                    turnover += Math.abs(dq);
                }
                while (mi < ours.metalContacts.size() && ours.metalContacts.get(mi) < t + DT) {
                    mi++;
                    if (p.cMetal > 0) {
                        double[] x = toMetal(bots[0], qInd[0], field, p);
                        led.c[Ledger.METAL] -= x[2];
                        turnover += Math.abs(x[2]);
                        record(res, t, K_METAL, x, p, bots[0], qInd[0]);
                    }
                }
            }
            if (on) {
                for (int i = (ours != null ? 1 : 0); i < nBots; i++) {
                    Bot b = bots[i];
                    if (rng.uniform() < pWall) {
                        if (rng.uniform() < p.metalFraction) {
                            turnover += Math.abs(toPerimeter(b, qInd[i], field, p)[2]);
                        }
                        else {
                            double dq = p.wallHitC * genFactor * Math.min(3, Math.max(0.1, p.wallHitSpeed.sample(rng)));
                            b.charge.addFrame(dq);
                            external += dq;
                            turnover += Math.abs(dq);
                        }
                        internal(b, qInd[i], p);
                    }
                    if (rng.uniform() < pIntake) {
                        double dq = p.intakeC * genFactor;
                        b.charge.addFrame(dq);
                        external += dq;
                        turnover += Math.abs(dq);
                    }
                    if (p.cMetal > 0 && rng.uniform() < pMetal) {
                        turnover += Math.abs(toMetal(b, qInd[i], field, p)[2]);
                        internal(b, qInd[i], p);
                    }
                    if (rng.uniform() < pContact) {
                        int j = i;
                        while (j == i || (ours != null && j == 0)) j = (int) (rng.uniform() * nBots);
                        turnover += Math.abs(between(b, qInd[i], bots[j], qInd[j])[2]);
                        internal(b, qInd[i], p);
                        internal(bots[j], qInd[j], p);
                    }
                }
            }
            for (int i = 0; i < nBots; i++) {
                Bot b = bots[i];
                double v = b.charge.vFrame(qInd[i]);
                double gC = Math.abs(v) > p.coronaV ? p.cFrame / p.coronaBleedS * (Math.abs(v) - p.coronaV) / Math.abs(v) : 0;
                double before = b.charge.qFrame() + b.charge.qElec();
                b.charge.step(DT, qInd[i], gW + gC, b.strap ? 1 / p.strapOhm : gIso);
                double lost = before - (b.charge.qFrame() + b.charge.qElec());
                drained += lost;
                turnover += Math.abs(lost);
                if (i == 0 && ours != null) {
                    led.c[Ledger.FLOOR] -= lost * gW / (gW + gC);
                    led.c[Ledger.AIR] -= lost * gC / (gW + gC);
                }
                if (i == 0 && ours != null) {
                    double e = internal(b, qInd[i], p);
                    if (e > 0) {
                        res.internalArcs++;
                        res.events.add(new Event(t, K_INTERNAL, p.internalBreakdownV, e, e, p.internalBreakdownV / p.sparkOhm));
                    }
                }
                else {
                    internal(b, qInd[i], p);
                }
            }
            drained += field.qPerimeter * (1 - perimFactor) + field.qMetal * (1 - metalFactor);
            field.qPerimeter *= perimFactor;
            field.qMetal *= metalFactor;
            if (s % 10 == 9) {
                double tb = field.tiles.total();
                field.tiles.decay(tileFactor);
                drained += tb - field.tiles.total();
                field.tiles.diffuse(tileD, 10 * DT, 2000);
            }
            Bot o = bots[0];
            double vf = o.charge.vFrame(qInd[0]);
            res.maxAbsKv = Math.max(res.maxAbsKv, Math.abs(vf) / 1000);
            if (recordSeries && t >= nextSeries) {
                res.series.add(new double[]{t, vf / 1000, (o.charge.vElec(qInd[0]) - vf) / 1000});
                nextSeries += 0.5;
            }
        }
        res.perimeterKvEnd = field.perimeterV(p) / 1000;
        res.tileUcEnd = field.tiles.total() * 1e6;
        if (ours != null) led.qEnd = bots[0].charge.qFrame() + bots[0].charge.qElec();
        double robotsEnd = 0;
        for (Bot b : bots) robotsEnd += b.charge.qFrame() + b.charge.qElec();
        res.globalResidualC = (robotsEnd + field.total() - globalStart) - (external - drained);
        res.globalTurnoverC = turnover;
        return res;
    }

    private static double[] toPerimeter(Bot b, double qInd, Field f, Params p) {

        double vf = b.charge.vFrame(qInd), vp = f.qPerimeter / p.cPerim;
        double d = (vf - vp) / (b.charge.frameElastance() + 1 / p.cPerim);
        b.charge.addFrame(-d);
        f.qPerimeter += d;
        return new double[]{vf - vp, 0.5 * d * (vf - vp), d};
    }

    private static double[] toMetal(Bot b, double qInd, Field f, Params p) {

        double vf = b.charge.vFrame(qInd), vm = f.qMetal / p.cMetal;
        double d = (vf - vm) / (b.charge.frameElastance() + 1 / p.cMetal);
        b.charge.addFrame(-d);
        f.qMetal += d;
        return new double[]{vf - vm, 0.5 * d * (vf - vm), d};
    }

    private static double[] between(Bot a, double qa, Bot b, double qb) {

        double va = a.charge.vFrame(qa), vb = b.charge.vFrame(qb);
        double d = (va - vb) / (a.charge.frameElastance() + b.charge.frameElastance());
        a.charge.addFrame(-d);
        b.charge.addFrame(d);
        return new double[]{va - vb, 0.5 * d * (va - vb), d};
    }

    private static double internal(Bot b, double qInd, Params p) {

        double diff = b.charge.vElec(qInd) - b.charge.vFrame(qInd);
        if (Math.abs(diff) <= p.internalBreakdownV) return 0;
        return b.charge.equalizeInternal(qInd);
    }

    private static void record(MatchResult res, double t, int kind, double[] dvE, Params p, Bot b, double qInd) {

        double e = Math.abs(dvE[1]);
        if (e > 1e-9) res.events.add(new Event(t, kind, dvE[0], e, p.coupling * e, Math.abs(dvE[0]) / p.sparkOhm));
        double ei = internal(b, qInd, p);
        if (ei > 0) {
            res.internalArcs++;
            res.events.add(new Event(t, K_INTERNAL, p.internalBreakdownV, ei, ei, p.internalBreakdownV / p.sparkOhm));
        }
    }
}
