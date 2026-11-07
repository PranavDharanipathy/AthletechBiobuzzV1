package org.athletech.sim.scenarios;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.MotorSpec;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robotcode.SimClock;
import org.athletech.sim.robotcode.SimFlywheelMotor;
import org.athletech.sim.utility.Csv;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//code in the loop: the robot's real Flywheel + ShotGate with TeamCode's FlywheelConstants, sim flywheel + battery
public final class FlywheelCodeScenario {

    private FlywheelCodeScenario() {}

    static final double H = 0.0005;

    static final class Setup {

        Flywheel.Params physics;
        double batteryVolts, batteryOhms, batteryDrop, otherCurrent, ballMass, exitRatio, energyLoss, feedInterval, velocityWindow, ticksPerRev;
        int balls;
        CodeLoop loop;
    }

    static Setup setup(RobotParams rp, SimContext ctx) {

        double t = rp.req("launcher.motor_stall_torque_nm"), is = rp.req("launcher.motor_stall_current_a"), w = rp.req("launcher.motor_free_speed_rpm"), i0 = rp.req("launcher.motor_free_current_a"), v = rp.opt("launcher.motor_nominal_v", 12);
        double dia = rp.req("launcher.flywheel_diameter_m"), moi = rp.req("launcher.flywheel_moi_kgm2"), count = rp.req("launcher.motor_count"), gear = rp.req("launcher.gear_ratio");
        double friction = rp.opt("launcher.friction_torque_nm", 0.0);
        Setup s = new Setup();
        s.batteryVolts = rp.req("battery.open_circuit_v");
        s.batteryOhms = rp.req("battery.internal_resistance_ohm");
        s.batteryDrop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        s.otherCurrent = rp.electronicsCurrent();
        s.ballMass = rp.req("ball.pollen.mass_kg");
        s.exitRatio = rp.req("launcher.pollen.exit_speed_ratio");
        s.energyLoss = rp.opt("launcher.pollen.energy_loss_factor", 1.6);
        s.feedInterval = rp.req("launcher.feed_interval_s");
        s.balls = (int) Math.max(1, Math.round(rp.opt("robot.max_pollen_held", 4)));
        s.velocityWindow = 1 / rp.opt("launcher.hub_velocity_hz", 50);
        s.ticksPerRev = FlywheelConstants.FLYWHEEL_TICKS_PER_REV;
        rp.requireAll();
        s.physics = new Flywheel.Params(moi, dia / 2, gear, (int) Math.round(count), MotorSpec.fromRpm(t, is, w, i0, v), friction, 0, true);
        s.loop = CodeLoop.from(ctx.args, rp, ctx.seed);
        return s;
    }

    static final class Rig implements AutoCloseable {

        final SimClock clock = new SimClock();
        final SimFlywheelMotor motor;
        final Battery battery;
        final org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel code;
        final Setup s;
        final CodeLoop loop;
        double t = 0, next = 0;

        Rig(Setup s, double batteryVolts, long seed) {

            this.s = s;
            battery = new Battery(batteryVolts, s.batteryOhms, s.batteryDrop);
            motor = new SimFlywheelMotor(new Flywheel(s.physics), s.ticksPerRev, s.velocityWindow);
            HardwareMap map = new HardwareMap();
            map.simVoltageSensor = () -> battery.lastVolts() > 0 ? battery.lastVolts() : battery.openCircuitVolts();
            code = new org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel(motor);
            code.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
            code.initVoltageSensor(map);
            loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed);
            clock.set(0);
        }

        boolean tick() {

            boolean ran = false;
            if (t >= next) {
                next += loop.next();
                clock.set(t);
                code.update();
                ran = true;
            }
            motor.step(t, H, battery, s.otherCurrent);
            t += H;
            return ran;
        }

        double ticksPerSecond() {
            return motor.physics().rpm() / 60 * s.physics.gearRatio() * s.ticksPerRev;
        }

        double fire() {
            return motor.physics().fire(s.ballMass, s.exitRatio, s.energyLoss);
        }

        @Override
        public void close() {
            clock.close();
        }
    }

    static final class SpinUp {

        double target, settleS = Double.NaN, overshoot, rippleRms, saturated, holdCurrent;
    }

    static SpinUp spinUp(Setup s, double target, double volts, List<double[]> trace, long seed) {

        SpinUp r = new SpinUp();
        r.target = target;
        double band = FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR;
        try (Rig rig = new Rig(s, volts, seed)) {
            rig.code.setVelocity(target, true);
            double lastOutside = 0, rip = 0, ripN = 0, sat = 0, loops = 0, current = 0, currentN = 0;
            while (rig.t < 20 && !(rig.t > 1 && rig.t - lastOutside > 2)) {
                boolean loop = rig.tick();
                double vel = rig.ticksPerSecond();
                if (Math.abs(vel - target) > band) {
                    lastOutside = rig.t;
                    rip = ripN = current = currentN = 0;
                }
                r.overshoot = Math.max(r.overshoot, vel - target);
                if (loop) {
                    loops++;
                    if (Math.abs(rig.code.getPower()) >= 0.999) sat++;
                    if (trace != null) trace.add(new double[] {rig.t, target, vel, rig.code.getCurrentVelocity(), rig.code.getRawVelocity(), rig.code.getPower(), rig.code.p, rig.code.i, rig.code.d, rig.code.v, rig.code.s, rig.battery.lastVolts()});
                }
                if (rig.t - lastOutside > 0.5) {
                    rip += (vel - target) * (vel - target);
                    ripN++;
                    current += rig.battery.lastCurrent();
                    currentN++;
                }
            }
            r.settleS = rig.t - lastOutside > 2 ? lastOutside : Double.NaN;
            r.rippleRms = ripN > 0 ? Math.sqrt(rip / ripN) : 0;
            r.saturated = loops > 0 ? sat / loops : 0;
            r.holdCurrent = currentN > 0 ? current / currentN : 0;
        }
        return r;
    }

    static final class Burst {

        final List<Double> exitSpeeds = new ArrayList<>(), feedErrors = new ArrayList<>();
        double durationS, dropPerShot, recoveryS = Double.NaN;
    }

    /// trigger held: blocker open while the ShotGate says ready (always without it), a ball every feed interval while open
    static Burst burst(Setup s, double target, double volts, boolean gate, List<double[]> trace, long seed) {

        Burst b = new Burst();
        ShotGate shotGate = new ShotGate(FlywheelConstants.SHOT_GATE_ENTER_BAND_BELOW, FlywheelConstants.SHOT_GATE_ENTER_BAND_ABOVE, FlywheelConstants.SHOT_GATE_HOLD_BAND_BELOW, FlywheelConstants.SHOT_GATE_HOLD_BAND_ABOVE, FlywheelConstants.SHOT_GATE_DWELL, FlywheelConstants.SHOT_GATE_MIN_OPEN, FlywheelConstants.SHOT_GATE_PULSE);
        double travel = 0.08; //blocker to the wheel
        double[] contacts = new double[s.balls];
        try (Rig rig = new Rig(s, volts, seed)) {
            rig.code.setVelocity(target, true);
            double band = FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR, inside = 0;
            while (rig.t < 20 && rig.t - inside < 1) {
                rig.tick();
                if (Math.abs(rig.ticksPerSecond() - target) > band) inside = rig.t;
            }
            double start = rig.t, lastEntry = -1e9, firstDrop = Double.NaN, lastShot = Double.NaN;
            int entered = 0, fired = 0;
            boolean open = false;
            while (rig.t < start + 10 && fired < s.balls) {
                if (rig.tick()) {
                    boolean ready = !gate || shotGate.update(rig.t, target, rig.code.getCurrentVelocity(), open);
                    open = ready;
                    if (open && entered < s.balls && rig.t - lastEntry >= s.feedInterval) {
                        contacts[entered++] = rig.t + travel;
                        lastEntry = rig.t;
                    }
                    if (trace != null) trace.add(new double[] {rig.t - start, target, rig.ticksPerSecond(), rig.code.getCurrentVelocity(), rig.code.getPower(), open ? 1 : 0});
                }
                if (fired < entered && rig.t >= contacts[fired]) {
                    double before = rig.ticksPerSecond();
                    b.feedErrors.add(before - target);
                    b.exitSpeeds.add(rig.fire());
                    if (fired == 0) firstDrop = before - rig.ticksPerSecond();
                    fired++;
                    lastShot = rig.t;
                }
            }
            b.durationS = rig.t - start;
            b.dropPerShot = firstDrop;
            while (rig.t < lastShot + 3) {
                rig.tick();
                if (Double.isNaN(b.recoveryS) && Math.abs(rig.ticksPerSecond() - target) <= band) b.recoveryS = rig.t - lastShot;
            }
        }
        return b;
    }

    static double rpm(double ticksPerSecond, Setup s) {
        return ticksPerSecond * 60 / (s.ticksPerRev * s.physics.gearRatio());
    }

    static double std(List<Double> xs) {

        double m = 0;
        for (double x : xs) m += x;
        m /= Math.max(1, xs.size());
        double v = 0;
        for (double x : xs) v += (x - m) * (x - m);
        return xs.size() > 1 ? Math.sqrt(v / (xs.size() - 1)) : 0;
    }

    public static void run(SimContext ctx) throws IOException, java.sql.SQLException {

        RobotParams rp = new RobotParams(ctx.db);
        Setup s = setup(rp, ctx);
        List<Double> table = ShooterConstants.FLYWHEEL_VELOCITIES;
        double low = table.get(0), high = table.get(table.size() - 1), mid = ShooterConstants.DEFAULT_FLYWHEEL_VELOCITY;
        double tired = Math.min(s.batteryVolts, 12.0);

        ctx.log.println("== Flywheel (robot's real Flywheel + ShotGate with TeamCode's FlywheelConstants, sim flywheel + battery) ==");
        ctx.log.printf(Locale.ROOT, "  sim flywheel: MOI %s kg*m^2, %d motor(s), gear %s, battery %.2f V / %.3f ohm, hub velocity over %.0f ms; loop %s%n", Csv.fmt(s.physics.moiKgm2()), s.physics.motorCount(), Csv.fmt(s.physics.gearRatio()), s.batteryVolts, s.batteryOhms, 1000 * s.velocityWindow, s.loop.describe());

        ctx.log.println("  spin-up from rest (settle = stays within +/-" + Csv.fmt(FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR) + " ticks/s):");
        SpinUp hold = null;
        for (double v : new double[] {s.batteryVolts, tired}) {
            for (double target : new double[] {low, mid, high}) {
                SpinUp r = spinUp(s, target, v, null, ctx.seed);
                if (v == s.batteryVolts && target == high) hold = r;
                ctx.log.printf(Locale.ROOT, "    %.2f V, %4.0f ticks/s (%4.0f rpm): settle %.2f s, overshoot %.0f ticks/s, ripple %.1f ticks/s RMS, power maxed %.0f%% of loops%n", v, target, rpm(target, s), r.settleS, r.overshoot, r.rippleRms, 100 * r.saturated);
            }
        }

        ctx.log.println("  burst of " + s.balls + " at " + Csv.fmt(high) + " ticks/s (exit speed spread is what changes the shot):");
        Burst gateBurst = null, plainBurst = null;
        for (double v : new double[] {s.batteryVolts, tired}) {
            for (boolean gate : new boolean[] {false, true}) {
                Burst b = burst(s, high, v, gate, null, ctx.seed);
                if (v == s.batteryVolts) {
                    if (gate) gateBurst = b;
                    else plainBurst = b;
                }
                ctx.log.printf(Locale.ROOT, "    %.2f V, %-28s %.2f s, exit speed %.2f m/s +/- %.3f, speed at feed %+.0f +/- %.0f ticks/s, drop %.0f ticks/s per shot, recovered in %.2f s%n", v, gate ? "ShotGate (robot settings):" : "feed every " + Csv.fmt(s.feedInterval) + " s:", b.durationS, mean(b.exitSpeeds), std(b.exitSpeeds), mean(b.feedErrors), std(b.feedErrors), b.dropPerShot, b.recoveryS);
            }
        }

        Path dir = ctx.outDir;
        Files.createDirectories(dir);
        List<double[]> spinTrace = new ArrayList<>(), burstTrace = new ArrayList<>();
        spinUp(s, high, s.batteryVolts, spinTrace, ctx.seed);
        burst(s, high, s.batteryVolts, FlywheelConstants.SHOT_GATE_ENABLED, burstTrace, ctx.seed);
        write(dir.resolve("flywheel_spinup_trace.csv"), "t_s,target,true_velocity,filtered_velocity,hub_velocity,power,p,i,d,v,s,bus_volts", spinTrace);
        write(dir.resolve("flywheel_burst_trace.csv"), "t_s,target,true_velocity,filtered_velocity,power,feeding", burstTrace);
        ctx.log.println("  traces: " + dir.resolve("flywheel_spinup_trace.csv").toAbsolutePath() + ", flywheel_burst_trace.csv");

        SimContext.Saver save = ctx.saver(rp, "sim flywheel");
        Burst used = FlywheelConstants.SHOT_GATE_ENABLED ? gateBurst : plainBurst;
        save.param("launcher.spinup_time_s", hold.settleS);
        save.param("launcher.hold_current_a", hold.holdCurrent);
        save.param("launcher.pollen.rpm_drop_per_shot_rpm", rpm(used.dropPerShot, s));
        save.param("launcher.pollen.recovery_time_s", used.recoveryS);
        save.report();
        if (!rp.placeholderInputs().isEmpty()) ctx.log.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
        if (!rp.assumptions().isEmpty()) ctx.log.println("  assumptions: " + String.join("; ", rp.assumptions()));
    }

    static double mean(List<Double> xs) {

        double m = 0;
        for (double x : xs) m += x;
        return xs.isEmpty() ? Double.NaN : m / xs.size();
    }

    static void write(Path file, String header, List<double[]> rows) throws IOException {

        StringBuilder b = new StringBuilder(header).append('\n');
        for (double[] r : rows) {
            for (int k = 0; k < r.length; k++) b.append(k == 0 ? "" : ",").append(Csv.fmt(r[k]));
            b.append('\n');
        }
        Files.write(file, b.toString().getBytes(StandardCharsets.UTF_8));
    }
}
