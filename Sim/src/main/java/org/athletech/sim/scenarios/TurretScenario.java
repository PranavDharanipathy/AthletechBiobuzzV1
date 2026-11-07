package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.robot.CrServoTurret;
import org.athletech.sim.utility.Csv;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.LanyardMap;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.firstinspires.ftc.teamcode.Tuners.TurretAutotuneEngine;
import org.firstinspires.ftc.teamcode.Tuners.TurretLanyardSweep;
import org.firstinspires.ftc.teamcode.Utility.Encoder;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

//code in the loop: the robot's real TurretAutotuneEngine + TurretController against a sim CR-servo turret
public final class TurretScenario {

    private TurretScenario() {}

    static final double H = 0.0005;
    static final double[] TURN_ANGLES = {2, 5, 10, 20, 30, 45, 60, 90};
    static final double[] MATCH_SPINS = {1.5, 3, 6}; //rad/s

    static final class Setup {

        CrServoTurret.Params plant;
        double tpd;
        CodeLoop loop;
        double shakeSync, shakeHz, shakeBand;
        boolean rejection = true;
        double[][] lanyards = new double[0][]; //degrees: taut at, side, V/deg, max V, drag V
        LanyardMap map;
        boolean learn;
        double startTicks;

        Setup withLanyards(double[][] lanyards, LanyardMap map, boolean learn, double startTicks) {

            Setup c = with(shakeSync, shakeHz, shakeBand, rejection);
            c.lanyards = lanyards;
            c.map = map;
            c.learn = learn;
            c.startTicks = startTicks;
            return c;
        }

        CrServoTurret turret(long seed) {

            CrServoTurret t = new CrServoTurret(plant, seed);
            for (double[] l : lanyards) t.addLanyard(l[0] * tpd, l[1], l[2] / tpd, l[3], l[4]);
            t.place(startTicks);
            return t;
        }

        Setup with(double syncTicks, double hz, double bandTicks, boolean rejection) {

            Setup c = new Setup();
            c.plant = plant;
            c.tpd = tpd;
            c.loop = loop;
            c.shakeSync = syncTicks;
            c.shakeHz = hz;
            c.shakeBand = bandTicks;
            c.rejection = rejection;
            c.lanyards = lanyards;
            c.map = map;
            c.learn = learn;
            c.startTicks = startTicks;
            return c;
        }
    }

    static Setup setup(RobotParams rp, SimContext ctx) {

        double tpd = rp.req("turret.ticks_per_degree");
        double gain = rp.req("turret.servo_speed_dps_per_v"), tau = rp.req("turret.servo_time_constant_s"), dead = rp.req("turret.servo_dead_time_s");
        double kinetic = rp.req("turret.kinetic_friction_v"), breakaway = rp.req("turret.breakaway_friction_v");
        double noise = rp.opt("turret.velocity_noise_dps", 0.44);
        double top = rp.opt("turret.max_speed_dps", Double.POSITIVE_INFINITY), accel = rp.opt("turret.max_accel_dps2", Double.POSITIVE_INFINITY);
        rp.requireAll();
        Setup s = new Setup();
        s.tpd = tpd;
        s.plant = new CrServoTurret.Params(gain * tpd, tau, dead, kinetic, breakaway, noise * tpd, top * tpd, accel * tpd);
        s.plant.startDelay = Math.max(dead, rp.opt("turret.servo_start_delay_s", dead));
        s.plant.coastDecel = rp.opt("turret.coast_decel_dps2", Double.POSITIVE_INFINITY) * tpd;
        s.plant.neutralDecel = rp.opt("turret.neutral_decel_dps2", Double.POSITIVE_INFINITY) * tpd;
        s.loop = CodeLoop.from(ctx.args, rp, ctx.seed);
        return s;
    }

    static TurretAutotuneEngine.Parameters robotNow() {
        return new TurretAutotuneEngine.Parameters(TurretConstants.TURRET_GAIN, TurretConstants.TURRET_TIME_CONSTANT, TurretConstants.TURRET_KINETIC_FRICTION, TurretConstants.TURRET_BREAKAWAY_FRICTION, TurretConstants.TURRET_DEAD_TIME, TurretConstants.TURRET_BANDWIDTH, TurretConstants.TURRET_DAMPING, TurretConstants.TURRET_INTEGRAL_RATIO, TurretConstants.TURRET_FRICTION_SCALE).capability(TurretConstants.TURRET_MAX_SPEED, TurretConstants.TURRET_MAX_ACCELERATION).startDelay(TurretConstants.TURRET_START_DELAY).deceleration(TurretConstants.TURRET_MAX_DECELERATION).shakeBandwidth(TurretConstants.TURRET_MIN_BANDWIDTH);
    }

    /// what a perfect autotune would give for this sim turret
    static TurretAutotuneEngine.Parameters matched(Setup s, double bandwidth) {

        CrServoTurret.Params q = s.plant;
        double finite = Double.isInfinite(q.maxSpeed) ? 0 : q.maxSpeed, accel = Double.isInfinite(q.maxAcceleration) ? 0 : q.maxAcceleration, coast = Double.isInfinite(q.coastDecel) ? 0 : q.coastDecel;
        return new TurretAutotuneEngine.Parameters(q.gain, q.timeConstant, q.kineticFriction, q.breakawayFriction, q.deadTime, bandwidth, 0.9, 0.3, 1).capability(finite, accel).startDelay(q.startDelay).deceleration(coast);
    }

    //autotune driving: 4 s of turning, then 2 s stopped (like stopping to shoot), round and round
    static double drivingTime(double t) {
        return 4 * Math.floor(t / 6) + Math.min(t % 6, 4);
    }

    static boolean stopped(double t) {
        return t % 6 >= 4;
    }

    //two spins mixed so the turret sees reversals and fast swings

    static double heading(double t) {
        return 0.6 * Math.sin(2 * Math.PI * 0.25 * t) + 0.3 * Math.sin(2 * Math.PI * 0.7 * t);
    }

    static double headingRate(double t) {
        return 0.6 * 2 * Math.PI * 0.25 * Math.cos(2 * Math.PI * 0.25 * t) + 0.3 * 2 * Math.PI * 0.7 * Math.cos(2 * Math.PI * 0.7 * t);
    }

    static double headingAccel(double t) {
        return -0.6 * Math.pow(2 * Math.PI * 0.25, 2) * Math.sin(2 * Math.PI * 0.25 * t) - 0.3 * Math.pow(2 * Math.PI * 0.7, 2) * Math.sin(2 * Math.PI * 0.7 * t);
    }

    static Encoder newFilter() {

        Encoder e = new Encoder(null);
        e.setVelocityFilterParameters(TunedConstants.TURRET_FILTER_MIN_BANDWIDTH, TunedConstants.TURRET_FILTER_MAX_BANDWIDTH, TunedConstants.TURRET_FILTER_SCORE_MEMORY, TunedConstants.TURRET_FILTER_STALE_TIME);
        return e;
    }

    static final class Tune {

        TurretAutotuneEngine engine;
        double seconds, maxTravelDeg;
    }

    static Tune autotune(Setup s, CrServoTurret turret, TurretAutotuneEngine engine, boolean driving, double maxSeconds, long seed) {
        return autotune(s, turret, engine, driving, 0, maxSeconds, seed);
    }

    /// spin = rad/s the robot keeps turning on top of the back and forth, so it goes round and round
    static Tune autotune(Setup s, CrServoTurret turret, TurretAutotuneEngine engine, boolean driving, double spin, double maxSeconds, long seed) {

        Encoder filter = newFilter();
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed);
        Tune r = new Tune();
        r.engine = engine;
        double t = 0, next = 0;
        boolean pressed = false;
        while (t < maxSeconds) {
            double dt = drivingTime(t);
            if (driving) turret.setRobotAcceleration(stopped(t) ? 0 : headingAccel(dt) * 180 / Math.PI * s.tpd);
            if (t >= next) {
                next += loop.next();
                double velocity = filter.updateFilteredVelocity(t, turret.encoderVelocity());
                if (driving && !pressed && t > 1) {
                    engine.requestStart();
                    pressed = true;
                }
                double heading = heading(dt) + spin * dt;
                double rate = stopped(t) ? 0 : headingRate(dt) + spin;
                engine.update(t, turret.encoderPosition(), velocity, 12.5, driving ? Math.atan2(Math.sin(heading), Math.cos(heading)) : 0, driving ? rate : 0);
                turret.command(t, engine.getLeftVolts(), engine.getRightVolts());
                r.maxTravelDeg = Math.max(r.maxTravelDeg, Math.abs(turret.position()) / s.tpd);
                if (engine.isDone() || engine.isFaulted()) break;
            }
            turret.step(t, H);
            t += H;
        }
        r.seconds = t;
        return r;
    }

    static TurretController controller(TurretAutotuneEngine.Parameters p, double tpd) {

        TurretController c = new TurretController();
        p.apply(c);
        c.vibrationReference = TurretConstants.TURRET_VIBRATION_REFERENCE_DEGREES * tpd;
        c.vibrationDeadband = TurretConstants.TURRET_VIBRATION_DEADBAND_DEGREES * tpd;
        c.minimumBandwidthFraction = TurretConstants.TURRET_MIN_BANDWIDTH_FRACTION;
        c.minimumBandwidth = p.shakeBandwidth > 0 ? p.shakeBandwidth : p.bandwidth; //same as TurretBase: what DRIVING found, else never soften
        c.lanyardTicksPerDegree = tpd;
        c.lanyardLearnRate = TurretConstants.TURRET_LANYARD_LEARN_RATE;
        c.lanyardLearnBand = TurretConstants.TURRET_LANYARD_LEARN_BAND_DEGREES * tpd;
        c.lanyardMaxVolts = TurretConstants.TURRET_LANYARD_MAX_VOLTS;
        c.lanyardStuckBand = TurretConstants.TURRET_LANYARD_STUCK_BAND_DEGREES * tpd;
        c.setLimits(TurretConstants.TURRET_MAX_SPEED_FRACTION, TurretConstants.TURRET_INTEGRAL_ZONE_DEGREES * tpd, TurretConstants.TURRET_MAX_INTEGRAL_VOLTS, TurretConstants.TURRET_FRICTION_RAMP_DEGREES * tpd, TurretConstants.TURRET_MOVING_DEGREES_PER_SECOND * tpd, TurretConstants.TURRET_HOLD_TOLERANCE_DEGREES * tpd);
        c.accelerationFraction = TurretConstants.TURRET_ACCELERATION_FRACTION;
        c.resyncBand = TurretConstants.TURRET_RESYNC_DEGREES * tpd;
        c.brakeBand = TurretConstants.TURRET_BRAKE_DEGREES * tpd;
        c.landingEnabled = TurretConstants.TURRET_LANDING_ENABLED;
        c.brakeLearning = TurretConstants.TURRET_BRAKE_LEARNING;
        c.setTravel(ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * tpd, ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * tpd); //same as TurretBase
        return c;
    }

    interface Target {

        double position(double t);

        double velocity(double t);
    }

    static final class Track {

        double rmsDeg, maxOverDeg, settleS = Double.NaN, peakSpeedDps, holdShakeDps, aimRmsDeg, motionRmsDeg, chatterV;
        final List<double[]> trace = new ArrayList<>();
    }

    /// settleFrom >= 0: target is a step at that time, settle = time until it stays within 0.5 deg
    static Track track(Setup s, TurretAutotuneEngine.Parameters p, Target target, double seconds, boolean spinning, double measureFrom, double settleFrom, boolean keepTrace, long seed) {

        CrServoTurret turret = s.turret(seed);
        TurretController c = controller(p, s.tpd);
        c.vibrationRejection = s.rejection && TurretConstants.TURRET_VIBRATION_REJECTION;
        c.lanyards = s.map;
        c.lanyardLearning = s.learn;
        turret.setVibration(s.shakeSync, s.shakeHz, s.shakeBand);
        Encoder filter = newFilter();
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed + 7);
        Track r = new Track();
        double t = 0, next = 0, last = 0, sum = 0, n = 0, lastOutside = Double.NaN, shake = 0, shakeN = 0;
        double aimSq = 0, motionSq = 0, aimN = 0, lastVolts = 0, chatter = 0, chatterN = 0;
        double finalTarget = target.position(seconds);
        while (t < seconds) {
            if (spinning) turret.setRobotAcceleration(headingAccel(t) * 180 / Math.PI * s.tpd);
            if (t >= next) {
                double dt = t - last;
                last = t;
                next += loop.next();
                double position = turret.encoderPosition();
                double velocity = filter.updateFilteredVelocity(t, turret.encoderVelocity());
                double volts = c.update(dt, target.position(t), target.velocity(t), position, velocity, 0.95 * 12.5);
                turret.command(t, volts, volts);
                if (t >= measureFrom) {
                    chatter += (volts - lastVolts) * (volts - lastVolts);
                    chatterN++;
                }
                lastVolts = volts;
                double errorDeg = (position - target.position(t)) / s.tpd;
                if (t >= measureFrom) {
                    sum += errorDeg * errorDeg;
                    n++;
                }
                if (settleFrom >= 0 && t >= settleFrom) {
                    double beyond = Math.signum(finalTarget - target.position(settleFrom - 1e-6)) * (position - finalTarget) / s.tpd;
                    r.maxOverDeg = Math.max(r.maxOverDeg, beyond);
                    if (Math.abs(position - finalTarget) > 0.5 * s.tpd) lastOutside = t;
                    if (t - settleFrom > 0.6 && Math.abs(position - finalTarget) < 0.5 * s.tpd) {
                        shake += velocity * velocity;
                        shakeN++;
                    }
                }
                r.peakSpeedDps = Math.max(r.peakSpeedDps, Math.abs(turret.velocity()) / s.tpd);
                if (keepTrace) r.trace.add(new double[] {t, target.position(t) / s.tpd, c.getReference() / s.tpd, position / s.tpd, velocity / s.tpd, volts, c.p, c.i, c.d, c.f, c.v});
            }
            turret.step(t, H);
            if (t >= measureFrom) {
                double aimError = (turret.aim() - target.position(t)) / s.tpd, motion = (turret.position() - target.position(t)) / s.tpd;
                aimSq += aimError * aimError;
                motionSq += motion * motion;
                aimN++;
            }
            t += H;
        }
        r.rmsDeg = n > 0 ? Math.sqrt(sum / n) : Double.NaN;
        r.aimRmsDeg = aimN > 0 ? Math.sqrt(aimSq / aimN) : Double.NaN;
        r.motionRmsDeg = aimN > 0 ? Math.sqrt(motionSq / aimN) : Double.NaN;
        r.chatterV = chatterN > 0 ? Math.sqrt(chatter / chatterN) : 0;
        if (settleFrom >= 0) r.settleS = Double.isNaN(lastOutside) ? 0 : lastOutside - settleFrom;
        r.holdShakeDps = shakeN > 0 ? Math.sqrt(shake / shakeN) / s.tpd : 0;
        return r;
    }

    static Track step(Setup s, TurretAutotuneEngine.Parameters p, double degrees, boolean keepTrace, long seed) {

        final double ticks = degrees * s.tpd;
        Target t = new Target() {
            public double position(double time) {
                return time < 0.3 ? 0 : ticks;
            }

            public double velocity(double time) {
                return 0;
            }
        };
        return track(s, p, t, 0.3 + Math.max(1.5, 0.03 * degrees), false, 1e9, 0.3, keepTrace, seed);
    }

    static final double[] STEP_SEQUENCE = {30, -60, 150, -190, 0, 90, 60, -30, 195};
    static final double STEP_SECONDS = 2;

    static final class Steps {

        double stallSeconds, divergedFraction;
        final double[] reachSeconds = new double[STEP_SEQUENCE.length]; //to within 5 deg, NaN = never
    }

    /// STEP_SEQUENCE, STEP_SECONDS apart. Stalled = over 10 deg off and under 15 deg/s; diverged = over 15 deg off its own plan
    static Steps steps(Setup s, TurretAutotuneEngine.Parameters p, long seed) {

        Target target = new Target() {
            public double position(double t) {

                int k = (int) Math.floor(t / STEP_SECONDS);
                return k < 1 ? 0 : STEP_SEQUENCE[Math.min(k - 1, STEP_SEQUENCE.length - 1)] * s.tpd;
            }

            public double velocity(double t) {
                return 0;
            }
        };
        Track r = track(s, p, target, STEP_SECONDS * (STEP_SEQUENCE.length + 1), false, 0, -1, true, seed);
        Steps out = new Steps();
        java.util.Arrays.fill(out.reachSeconds, Double.NaN);
        double last = 0, diverged = 0, total = 0;
        for (double[] row : r.trace) {
            double t = row[0], dt = t - last;
            last = t;
            if (Math.abs(row[1] - row[3]) > 10 && Math.abs(row[4]) < 15) out.stallSeconds += dt;
            if (Math.abs(row[2] - row[3]) > 15) diverged += dt;
            total += dt;
            int k = (int) Math.floor(t / STEP_SECONDS) - 1;
            if (k >= 0 && k < STEP_SEQUENCE.length && Double.isNaN(out.reachSeconds[k]) && Math.abs(row[3] - STEP_SEQUENCE[k]) < 5) out.reachSeconds[k] = t - STEP_SECONDS * (k + 1);
        }
        out.divergedFraction = total > 0 ? diverged / total : 0;
        return out;
    }

    static Track hold(Setup s, TurretAutotuneEngine.Parameters p, long seed) {

        final double ticks = 20 * s.tpd;
        Target t = new Target() {
            public double position(double time) {
                return time < 0.3 ? 0 : ticks;
            }

            public double velocity(double time) {
                return 0;
            }
        };
        return track(s, p, t, 8, false, 2, -1, false, seed);
    }

    static final class MatchSpin {

        double maxTravelDeg, aimedFraction, firstSwingDeg = Double.NaN;
        int swings;
    }

    /// Shooter's match aim (TurretRoute picks the turn) holding a field direction while the robot spins at spin rad/s
    /// for half the time, then back the other way
    static MatchSpin matchSpin(Setup s, TurretAutotuneEngine.Parameters p, double spin, double seconds, long seed) {

        CrServoTurret turret = s.turret(seed);
        TurretController c = controller(p, s.tpd);
        TurretRoute route = new TurretRoute();
        Encoder filter = newFilter();
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed + 7);
        double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
        double toAim = ShooterConstants.TURRET_AIM_DIRECTION * -180 / Math.PI;
        MatchSpin r = new MatchSpin();
        double t = 0, next = 0, last = 0, aimed = 0, n = 0, half = seconds / 2;
        while (t < seconds) {
            turret.setRobotAcceleration(headingAccel(t) * 180 / Math.PI * s.tpd);
            if (t >= next) {
                double dt = t - last;
                last = t;
                next += loop.next();
                double aim = toAim * (heading(t) + spin * (t < half ? t : 2 * half - t));
                double aimRate = toAim * (headingRate(t) + (t < half ? spin : -spin));
                double position = turret.encoderPosition();
                double velocity = filter.updateFilteredVelocity(t, turret.encoderVelocity());
                double tt = route.update(aim, aimRate, position / s.tpd, min, max, ShooterConstants.TURRET_SWING_MARGIN_DEGREES);
                if (route.hasSwung() && r.swings++ == 0) r.firstSwingDeg = Math.abs(position / s.tpd);
                double volts = c.update(dt, tt * s.tpd, route.isOutside(min, max) ? 0 : aimRate * s.tpd, position, velocity, 0.95 * 12.5);
                turret.command(t, volts, volts);
                if (Math.abs(Math.IEEEremainder(position / s.tpd - aim, 360)) < 2) aimed++;
                n++;
            }
            turret.step(t, H);
            r.maxTravelDeg = Math.max(r.maxTravelDeg, Math.abs(turret.position()) / s.tpd);
            t += H;
        }
        r.aimedFraction = n > 0 ? aimed / n : 0;
        return r;
    }

    static Track spin(Setup s, TurretAutotuneEngine.Parameters p, boolean keepTrace, long seed) {

        final double scale = ShooterConstants.TURRET_AIM_DIRECTION * -180 / Math.PI * s.tpd;
        Target t = new Target() {
            public double position(double time) {
                return Math.max(-50 * s.tpd, Math.min(50 * s.tpd, scale * heading(time)));
            }

            public double velocity(double time) {
                return Math.abs(scale * heading(time)) > 50 * s.tpd ? 0 : scale * headingRate(time);
            }
        };
        return track(s, p, t, 20, true, 2, -1, keepTrace, seed);
    }

    public static void run(SimContext ctx) throws IOException, java.sql.SQLException {

        RobotParams rp = new RobotParams(ctx.db);
        Setup s = setup(rp, ctx);
        boolean verbose = ctx.args != null && ctx.args.flag("verbose");
        ctx.log.println("== Turret (robot's real TurretAutotuneEngine + TurretController, sim CR-servo turret) ==");
        ctx.log.printf(Locale.ROOT, "  sim turret: %.1f deg/s per V, lag %.0f ms, dead time %.0f ms (%.0f ms from rest), friction %.2f / %.2f V, top %.0f deg/s, accel %.0f deg/s^2, %.2f ticks/deg; loop %s%n", s.plant.gain / s.tpd, 1000 * s.plant.timeConstant, 1000 * s.plant.deadTime, 1000 * s.plant.startDelay, s.plant.kineticFriction, s.plant.breakawayFriction, s.plant.maxSpeed / s.tpd, s.plant.maxAcceleration / s.tpd, s.tpd, s.loop.describe());

        TurretAutotuneEngine.Parameters now = robotNow();
        Tune stationary = autotune(s, new CrServoTurret(s.plant, ctx.seed), new TurretAutotuneEngine(TurretAutotuneEngine.Mode.STATIONARY, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, now), false, 600, ctx.seed + 1);
        report("Turret Autotuner, STATIONARY", stationary, verbose, ctx);
        TurretAutotuneEngine.Parameters tuned = stationary.engine.isDone() ? stationary.engine.getParameters() : null;

        TurretAutotuneEngine.Parameters drove = null;
        if (tuned != null) {
            //driven with the flywheel on, like on the robot: that's what finds the rejection floor
            CrServoTurret shaken = new CrServoTurret(s.plant, ctx.seed + 2);
            double deg = rp.opt("turret.vibration_deg", 0.3), part = deg * s.tpd / Math.sqrt(2);
            shaken.setVibration(part, rp.opt("launcher.max_rpm", 3568) / 60, part);
            Tune driving = autotune(s, shaken, new TurretAutotuneEngine(TurretAutotuneEngine.Mode.DRIVING, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, tuned), true, 400, ctx.seed + 3);
            report("Turret Autotuner, DRIVING with the flywheel shaking it (from the stationary result)", driving, verbose, ctx);
            if (driving.engine.isDone()) drove = driving.engine.getParameters();
            Tune spinning = autotune(s, new CrServoTurret(s.plant, ctx.seed + 4), new TurretAutotuneEngine(TurretAutotuneEngine.Mode.DRIVING, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, tuned), true, 1.5, 400, ctx.seed + 5);
            report("Turret Autotuner, DRIVING while spinning round and round", spinning, verbose, ctx);
        }

        ctx.log.println();
        ctx.log.println("  how the controller does (30 deg step: overshoot / settle to 0.5 deg; robot spinning: RMS aim error):");
        Track nowStep = step(s, now, 30, false, ctx.seed), nowSpin = spin(s, now, false, ctx.seed);
        line(ctx, "robot's TurretConstants now", now, nowStep, nowSpin);
        Track bestStep = nowStep, bestSpin = nowSpin;
        TurretAutotuneEngine.Parameters best = now;
        if (tuned != null) {
            Track a = step(s, tuned, 30, false, ctx.seed), b = spin(s, tuned, false, ctx.seed);
            line(ctx, "after STATIONARY autotune", tuned, a, b);
            best = tuned;
            bestStep = a;
            bestSpin = b;
        }
        if (drove != null) {
            Track a = step(s, drove, 30, false, ctx.seed), b = spin(s, drove, false, ctx.seed);
            line(ctx, "after DRIVING autotune", drove, a, b);
            best = drove;
            bestStep = a;
            bestSpin = b;
        }

        List<double[]> turn = new ArrayList<>();
        StringBuilder turnText = new StringBuilder();
        for (double deg : TURN_ANGLES) {
            Track tr = step(s, best, deg, false, ctx.seed + (long) deg);
            turn.add(new double[] {deg, tr.settleS});
            turnText.append(String.format(Locale.ROOT, " %.0f deg %.2f s,", deg, tr.settleS));
        }
        ctx.log.println("  time to turn and settle (best values):" + turnText.substring(0, turnText.length() - 1));

        StringBuilder spinText = new StringBuilder();
        for (double rate : MATCH_SPINS) {
            MatchSpin m = matchSpin(s, best, rate, 30, ctx.seed + 11);
            spinText.append(String.format(Locale.ROOT, " %.1f rad/s: max travel %.0f deg, %d swings, aimed %.0f%%;", rate, m.maxTravelDeg, m.swings, 100 * m.aimedFraction));
        }
        ctx.log.println("  match aim, robot spinning round one way then back (aimed = within 2 deg):" + spinText.substring(0, spinText.length() - 1));

        vibration(s, rp, best, ctx);

        lanyards(s, best, ctx);

        if (ctx.args != null && ctx.args.flag("faults")) faults(s, ctx);

        Path dir = ctx.outDir;
        Files.createDirectories(dir);
        writeTrace(dir.resolve("turret_step_trace.csv"), step(s, best, 30, true, ctx.seed).trace);
        writeTrace(dir.resolve("turret_spin_trace.csv"), spin(s, best, true, ctx.seed).trace);
        ctx.log.println("  traces: " + dir.resolve("turret_step_trace.csv").toAbsolutePath() + ", turret_spin_trace.csv");

        SimContext.Saver save = ctx.saver(rp, "sim turret");
        if (tuned != null) save.param("turret.autotune_bandwidth_rad_s", (drove != null ? drove : tuned).bandwidth);
        save.param("turret.step_overshoot_deg", bestStep.maxOverDeg);
        save.param("turret.spin_tracking_error_deg", bestSpin.rmsDeg);
        save.param("turret.max_speed_dps", step(s, best, 90, false, ctx.seed).peakSpeedDps);
        save.curve("turret.time_to_turn", turn);
        save.report();
        if (!rp.placeholderInputs().isEmpty()) ctx.log.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
        if (!rp.assumptions().isEmpty()) ctx.log.println("  assumptions: " + String.join("; ", rp.assumptions()));
    }

    static void report(String title, Tune r, boolean verbose, SimContext ctx) {

        TurretAutotuneEngine e = r.engine;
        ctx.log.printf(Locale.ROOT, "  %s: %s after %.0f s (max travel %.0f deg)%n", title, e.isFaulted() ? "FAULT - " + e.getStatus() : e.isDone() ? "done" : "didn't finish, stuck in '" + e.getStage() + "'", r.seconds, r.maxTravelDeg);
        for (String w : e.getWarnings()) ctx.log.println("    warning: " + w);
        List<String> h = e.getHistory();
        for (int k = verbose ? 0 : Math.max(0, h.size() - 3); k < h.size(); k++) ctx.log.println("    " + h.get(k));
        if (e.isDone()) {
            Map<String, Double> m = e.getResults();
            ctx.log.printf(Locale.ROOT, "    -> TURRET_GAIN %s, TIME_CONSTANT %s, DEAD_TIME %s, KINETIC %s, BREAKAWAY %s, BANDWIDTH %s, INTEGRAL_RATIO %s, FRICTION_SCALE %s%s%n", m.get("gain"), m.get("time_constant"), m.get("dead_time"), m.get("kinetic_friction"), m.get("breakaway_friction"), m.get("bandwidth"), m.get("integral_ratio"), m.get("friction_scale"), m.containsKey("shake_bandwidth") ? ", SHAKE_BANDWIDTH " + m.get("shake_bandwidth") : "");
        }
    }

    static void line(SimContext ctx, String name, TurretAutotuneEngine.Parameters p, Track step, Track spin) {
        ctx.log.printf(Locale.ROOT, "    %-28s bandwidth %5.2f: step overshoot %.2f deg, settle %.2f s; spinning %.2f deg RMS, hold shake %.1f deg/s%n", name, p.bandwidth, step.maxOverDeg, step.settleS, spin.rmsDeg, step.holdShakeDps);
    }

    //flywheel shake: half at the flywheel's rotation rate, half in the frame's 8-14 Hz band
    static void vibration(Setup s, RobotParams rp, TurretAutotuneEngine.Parameters p, SimContext ctx) {

        double deg = rp.opt("turret.vibration_deg", 0.3), hz = rp.opt("launcher.max_rpm", 3568) / 60;
        double part = deg * s.tpd / Math.sqrt(2);
        ctx.log.println();
        ctx.log.printf(Locale.ROOT, "  flywheel on (turret shake %.2f deg RMS: %.0f Hz from the flywheel + 8-14 Hz frame), where the turret really points:%n", deg, hz);
        for (boolean rejection : new boolean[] {false, true}) {
            Setup v = s.with(part, hz, part, rejection);
            Track hold = hold(v, p, ctx.seed + 40), spin = spin(v, p, false, ctx.seed + 41);
            Track calm = hold(s.with(0, hz, 0, rejection), p, ctx.seed + 40);
            ctx.log.printf(Locale.ROOT, "    vibration rejection %-3s: holding %.2f deg RMS (servo-made %.2f, volts jumping %.2f V/loop), spinning %.2f deg RMS; flywheel off: holding %.2f deg%n", rejection ? "on" : "off", hold.aimRmsDeg, hold.motionRmsDeg, hold.chatterV, spin.aimRmsDeg, calm.aimRmsDeg);
        }
    }

    //made-up lanyards (the robot has none yet): two that overlap going up, one going down, one that pulls down over the whole range
    static final double[][] TEST_LANYARDS = {
            {40, 1, 0.03, 1.5, 0.1},
            {90, 1, 0.04, 2.0, 0.1},
            {-60, -1, 0.035, 2.0, 0.1},
            {-200, 1, 0.004, 0.8, 0}
    };
    static final double SWEEP_DPS = TurretConstants.TURRET_LANYARD_SWEEP_DEGREES_PER_SECOND, SWEEP_DWELL = TurretConstants.TURRET_LANYARD_SWEEP_DWELL, SWEEP_EDGE = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES - 5;

    static LanyardMap newMap() {
        return new LanyardMap(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES, ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES, TurretConstants.TURRET_LANYARD_BIN_DEGREES);
    }

    /// back and forth over the whole travel with a stop at each end, the same sweep the manual tuner's X runs
    static Target sweep(final double tpd) {

        return new Target() {
            public double position(double t) {
                return TurretLanyardSweep.position(t, -SWEEP_EDGE, SWEEP_EDGE, SWEEP_DPS, SWEEP_DWELL) * tpd;
            }

            public double velocity(double t) {
                return TurretLanyardSweep.velocity(t, -SWEEP_EDGE, SWEEP_EDGE, SWEEP_DPS, SWEEP_DWELL) * tpd;
            }
        };
    }

    static double sweepSeconds(int passes) {
        return passes * TurretLanyardSweep.period(-SWEEP_EDGE, SWEEP_EDGE, SWEEP_DPS, SWEEP_DWELL);
    }

    static final class LanyardResult {

        double sweepRmsDeg, holdRmsDeg, holdWorstDeg, spinRmsDeg, mapErrorV;
    }

    static final double[] HOLD_ANGLES = {170, 120, 60, -100, -175};

    static LanyardResult lanyardTest(Setup s, TurretAutotuneEngine.Parameters p, LanyardMap map, long seed) {
        return lanyardTest(s, p, map, TEST_LANYARDS, false, seed);
    }

    /// learn = keeps learning during the holds and the spin, like TeleOp does
    static LanyardResult lanyardTest(Setup s, TurretAutotuneEngine.Parameters p, LanyardMap map, double[][] lanyards, boolean learn, long seed) {

        LanyardResult r = new LanyardResult();
        Setup off = s.withLanyards(lanyards, map, false, 0);
        r.sweepRmsDeg = track(off, p, sweep(s.tpd), sweepSeconds(1), false, 0.5, -1, false, seed).rmsDeg;
        double sq = 0;
        for (double angle : HOLD_ANGLES) {
            final double ticks = angle * s.tpd;
            Target hold = new Target() {
                public double position(double t) {
                    return ticks;
                }

                public double velocity(double t) {
                    return 0;
                }
            };
            Track h = track(s.withLanyards(lanyards, map, learn, ticks - Math.signum(angle) * 10 * s.tpd), p, hold, 5, false, 2, -1, false, seed + (long) angle);
            sq += h.rmsDeg * h.rmsDeg;
            r.holdWorstDeg = Math.max(r.holdWorstDeg, h.rmsDeg);
        }
        r.holdRmsDeg = Math.sqrt(sq / HOLD_ANGLES.length);
        final double scale = ShooterConstants.TURRET_AIM_DIRECTION * -180 / Math.PI * s.tpd, center = 120 * s.tpd;
        Target spin = new Target() {
            public double position(double t) {
                return center + scale * heading(t);
            }

            public double velocity(double t) {
                return scale * headingRate(t);
            }
        };
        r.spinRmsDeg = track(s.withLanyards(lanyards, map, learn, center), p, spin, 20, true, 2, -1, false, seed + 5).rmsDeg;
        if (map != null && lanyards.length > 0) {
            CrServoTurret truth = s.withLanyards(lanyards, null, false, 0).turret(seed);
            double err = 0;
            int n = 0;
            for (int k = 0; k < map.size(); k++) {
                double deg = map.binDegrees(k);
                if (Math.abs(deg) > SWEEP_EDGE - 10) continue;
                double up = -truth.lanyardVolts(deg * s.tpd, 1), down = -truth.lanyardVolts(deg * s.tpd, -1);
                err += Math.pow(map.forwardVolts(k) - up, 2) + Math.pow(map.backwardVolts(k) - down, 2);
                n += 2;
            }
            r.mapErrorV = Math.sqrt(err / n);
        }
        return r;
    }

    /// sweeps with learning on, returns the map it learned
    static LanyardMap learnLanyards(Setup s, TurretAutotuneEngine.Parameters p, double[][] lanyards, int passes, long seed) {

        LanyardMap map = newMap();
        track(s.withLanyards(lanyards, map, true, 0), p, sweep(s.tpd), sweepSeconds(passes), false, 1e9, -1, false, seed);
        return map;
    }

    static void lanyards(Setup s, TurretAutotuneEngine.Parameters p, SimContext ctx) {

        ctx.log.println();
        ctx.log.println("  lanyards (made up, 4 overlapping, up to " + String.format(Locale.ROOT, "%.1f", maxPull(s)) + " V of pull), what the X sweep in the manual tuner learns:");
        lanyardLine(ctx, "no lanyards at all", lanyardTest(s, p, null, new double[0][], false, ctx.seed + 60));
        LanyardResult off = lanyardTest(s, p, null, ctx.seed + 60);
        lanyardLine(ctx, "compensation off", off);
        for (int passes : new int[] {1, 3, 8}) {
            LanyardMap map = learnLanyards(s, p, TEST_LANYARDS, passes, ctx.seed + 61);
            lanyardLine(ctx, "learned, " + passes + " sweep" + (passes > 1 ? "s" : ""), lanyardTest(s, p, map, ctx.seed + 60));
            if (passes == 8) {
                lanyardLine(ctx, "  + still learning", lanyardTest(s, p, map, TEST_LANYARDS, true, ctx.seed + 63));
                lanyardLine(ctx, "  then again", lanyardTest(s, p, map, ctx.seed + 64));
            }
            if (ctx.args != null && ctx.args.flag("verbose") && passes == 8) {
                CrServoTurret truth = s.withLanyards(TEST_LANYARDS, null, false, 0).turret(1);
                for (int k = 0; k < map.size(); k += 4) ctx.log.printf(Locale.ROOT, "      %6.0f deg: up %5.2f V (real %5.2f), down %5.2f V (real %5.2f)%n", map.binDegrees(k), map.forwardVolts(k), -truth.lanyardVolts(map.binDegrees(k) * s.tpd, 1), map.backwardVolts(k), -truth.lanyardVolts(map.binDegrees(k) * s.tpd, -1));
            }
        }
        LanyardMap none = learnLanyards(s, p, new double[0][], 3, ctx.seed + 62);
        ctx.log.printf(Locale.ROOT, "    no lanyards, learning on: map stays at %.2f V most%n", none.largestVolts());
    }

    static double maxPull(Setup s) {

        CrServoTurret t = s.withLanyards(TEST_LANYARDS, null, false, 0).turret(1);
        double m = 0;
        for (double deg = -SWEEP_EDGE; deg <= SWEEP_EDGE; deg += 1) m = Math.max(m, Math.abs(t.lanyardVolts(deg * s.tpd, 0)));
        return m;
    }

    static void lanyardLine(SimContext ctx, String name, LanyardResult r) {
        ctx.log.printf(Locale.ROOT, "    %-22s sweep %.2f deg RMS, holds %.2f deg RMS (worst %.2f), spinning at 120 deg %.2f deg RMS%s%n", name, r.sweepRmsDeg, r.holdRmsDeg, r.holdWorstDeg, r.spinRmsDeg, r.mapErrorV > 0 ? String.format(Locale.ROOT, ", map off by %.2f V", r.mapErrorV) : "");
    }

    static void faults(Setup s, SimContext ctx) {

        ctx.log.println();
        ctx.log.println("  autotuner diagnostics against broken turrets:");
        String[] names = {"encoder reversed", "right servo reversed (fighting)", "left servo dead", "turret jammed"};
        for (int k = 0; k < names.length; k++) {
            CrServoTurret t = new CrServoTurret(s.plant, ctx.seed + 20 + k);
            if (k == 0) t.encoderSign = -1;
            if (k == 1) t.rightSign = -1;
            if (k == 2) t.leftStrength = 0;
            if (k == 3) t.jammed = true;
            Tune r = autotune(s, t, new TurretAutotuneEngine(TurretAutotuneEngine.Mode.STATIONARY, s.tpd, ShooterConstants.TURRET_AIM_DIRECTION, robotNow()), false, 600, ctx.seed + 30 + k);
            TurretAutotuneEngine e = r.engine;
            ctx.log.printf(Locale.ROOT, "    %-32s %s%s%n", names[k] + ":", e.isFaulted() ? "FAULT - " + e.getStatus() : e.isDone() ? "done" : "didn't finish", e.getWarnings().isEmpty() ? "" : " (warning: " + String.join("; ", e.getWarnings()) + ")");
        }
    }

    static void writeTrace(Path file, List<double[]> rows) throws IOException {

        StringBuilder b = new StringBuilder("t_s,target_deg,reference_deg,position_deg,velocity_dps,volts,p,i,d,f,v\n");
        for (double[] r : rows) {
            for (int k = 0; k < r.length; k++) b.append(k == 0 ? "" : ",").append(Csv.fmt(r[k]));
            b.append('\n');
        }
        Files.write(file, b.toString().getBytes(StandardCharsets.UTF_8));
    }
}
