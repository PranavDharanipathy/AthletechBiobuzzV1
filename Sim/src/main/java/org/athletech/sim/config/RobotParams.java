package org.athletech.sim.config;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.MotorSpec;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.robot.Turret;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.undeterminables.OdometryErrorModel;
import org.athletech.sim.utility.Vec3;

import java.sql.SQLException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

/**
 * Reads the profile into model parameters, keeping track of what was missing, what was assumed
 * and which inputs were only placeholders. Sims call {@link #requireAll()} after building their
 * models; outputs inherit {@link #outputKind()} (placeholder in, placeholder out).
 */
public final class RobotParams {
    private final ProfileDb db;
    private final List<String> missing = new ArrayList<>();
    private final List<String> assumptions = new ArrayList<>();
    private final Set<String> placeholders = new TreeSet<>();

    public RobotParams(ProfileDb db) {
        this.db = db;
    }

    public ProfileDb db() { return db; }
    public List<String> missing() { return missing; }
    public List<String> assumptions() { return assumptions; }
    public Set<String> placeholderInputs() { return placeholders; }

    /** A required number; recorded as missing (and NaN returned) when empty. */
    public double req(String key) {
        ProfileDb.ParamRow r = row(key);
        if (r == null || r.value() == null) {
            if (!missing.contains(key)) missing.add(key);
            return Double.NaN;
        }
        note(key, r.sourceKind());
        return r.value();
    }

    /** An optional number with a documented fallback. */
    public double opt(String key, double fallback) {
        ProfileDb.ParamRow r = row(key);
        if (r == null || r.value() == null) {
            if (!Double.isNaN(fallback)) assumptions.add(key + " not set: using " + fmt(fallback));
            return fallback;
        }
        note(key, r.sourceKind());
        return r.value();
    }

    public boolean flag(String key, boolean fallback) {
        double v = opt(key, fallback ? 1 : 0);
        return v >= 0.5;
    }

    public String text(String key) {
        ProfileDb.ParamRow r = row(key);
        return r == null ? null : r.text();
    }

    /** An error distribution; missing ones are treated as "no error" and noted. */
    public Distribution dist(String name) {
        try {
            ProfileDb.DistRow r = db.dist(name);
            if (r == null || !r.hasData()) {
                assumptions.add(name + " not set: treated as zero error (results are optimistic)");
                return Distribution.none(name, r == null ? "" : r.unit());
            }
            note(name, r.sourceKind());
            return r.toDistribution();
        } catch (SQLException e) {
            throw new IllegalStateException(e);
        }
    }

    private ProfileDb.ParamRow row(String key) {
        try {
            return db.param(key);
        } catch (SQLException e) {
            throw new IllegalStateException(e);
        }
    }

    private void note(String key, SourceKind kind) {
        if (kind == SourceKind.PLACEHOLDER) placeholders.add(key);
    }

    private static String fmt(double v) {
        return org.athletech.sim.utility.Csv.fmt(v);
    }

    /** Throws with a readable list if any required input was missing. */
    public void requireAll() {
        if (missing.isEmpty()) return;
        StringBuilder sb = new StringBuilder("The profile is missing values these sims need:\n");
        for (String k : missing) {
            Catalog.Param p = Catalog.param(k);
            sb.append("  ").append(k);
            if (p != null) sb.append(" [").append(p.unit()).append("]  ").append(p.description());
            sb.append('\n');
        }
        sb.append("Set them with:  profile set <key> <value>\n");
        sb.append("Or try the sims with example numbers:  profile init --example --db out/example_profile.db");
        throw new MissingInputsException(sb.toString(), java.util.Collections.unmodifiableList(new java.util.ArrayList<>(missing)));
    }

    public static final class MissingInputsException extends RuntimeException {
        private static final long serialVersionUID = 1L;
        public final transient List<String> keys;

        MissingInputsException(String msg, List<String> keys) {
            super(msg);
            this.keys = keys;
        }
    }

    /** Placeholder inputs make placeholder outputs, so they can never pass for real data. */
    public SourceKind outputKind() {
        return placeholders.isEmpty() ? SourceKind.SIM : SourceKind.PLACEHOLDER;
    }

    /** Text stored with each sim run in the sources table. */
    public String provenance(String command) {
        StringBuilder sb = new StringBuilder(command);
        if (!placeholders.isEmpty()) sb.append("\nPlaceholder inputs: ").append(String.join(", ", placeholders));
        if (!assumptions.isEmpty()) sb.append("\nAssumptions: ").append(String.join("; ", assumptions));
        return sb.toString();
    }

    // ------------------------------------------------------------------ model builders

    public MotorSpec driveMotor() {
        double t = req("drive.motor_stall_torque_nm"), is = req("drive.motor_stall_current_a"),
                w = req("drive.motor_free_speed_rpm"), i0 = req("drive.motor_free_current_a"), v = opt("drive.motor_nominal_v", 12);
        if (anyNaN(t, is, w, i0)) return null;
        return MotorSpec.fromRpm(t, is, w, i0, v);
    }

    public MecanumDrive.Params drive() {
        double mass = req("robot.mass_kg"), len = req("robot.length_m"), wid = req("robot.width_m");
        double moi = opt("robot.moi_kgm2", Double.NaN);
        if (Double.isNaN(moi) && !anyNaN(mass, len, wid)) {
            moi = mass * (len * len + wid * wid) / 12;
            assumptions.add("robot.moi_kgm2 not set: estimated " + fmt(moi) + " kg*m^2 from mass and size");
        }
        double wheelD = req("drive.wheel_diameter_m"), track = req("drive.track_width_m"), base = req("drive.wheelbase_m");
        double gear = req("drive.gear_ratio"), mu = req("drive.wheel_friction_coeff"), eta = req("drive.strafe_efficiency");
        double crr = opt("drive.rolling_resistance_coeff", 0.04);
        double limit = opt("drive.current_limit_a", Double.NaN);
        double maxPower = opt("drive.max_power", 1.0);
        boolean brake = flag("drive.zero_power_brake", true);
        MotorSpec motor = driveMotor();
        if (motor == null || anyNaN(mass, moi, wheelD, track, base, gear, mu, eta)) return null;
        return new MecanumDrive.Params(mass, moi, wheelD / 2, track, base, gear, motor, mu, eta, crr,
                Double.isNaN(limit) ? 0 : limit, maxPower, brake);
    }

    /** A fresh battery. {@code openCircuitV} overrides the profile's resting voltage when not NaN. */
    public Battery battery(double openCircuitV) {
        double voc = Double.isNaN(openCircuitV) ? req("battery.open_circuit_v") : openCircuitV;
        double r = req("battery.internal_resistance_ohm");
        double drop = opt("battery.voltage_drop_per_ah", 0.6);
        if (anyNaN(voc, r)) return null;
        return new Battery(voc, r, drop);
    }

    public double electronicsCurrent() {
        return opt("robot.electronics_current_a", 0.8);
    }

    public Projectile.Ball ball(String b) {
        double d = req("ball." + b + ".diameter_m"), m = req("ball." + b + ".mass_kg");
        double cd = opt("ball." + b + ".drag_coeff", 0.5), cl = opt("ball." + b + ".lift_coeff", 0.0);
        if (anyNaN(d, m)) return null;
        return new Projectile.Ball(b, m, d, cd, cl);
    }

    public Launcher.Params launcher() {
        double t = req("launcher.motor_stall_torque_nm"), is = req("launcher.motor_stall_current_a"),
                w = req("launcher.motor_free_speed_rpm"), i0 = req("launcher.motor_free_current_a"),
                v = opt("launcher.motor_nominal_v", 12);
        double dia = req("launcher.flywheel_diameter_m"), moi = req("launcher.flywheel_moi_kgm2");
        double count = req("launcher.motor_count"), gear = req("launcher.gear_ratio");
        double friction = opt("launcher.friction_torque_nm", 0.0), kp = req("launcher.kp_per_rpm");
        boolean comp = flag("launcher.voltage_compensation", false);
        double maxRpm = req("launcher.max_rpm"), tol = req("launcher.ready_tolerance_rpm"), feed = req("launcher.feed_interval_s");
        boolean waitReady = flag("launcher.wait_for_ready", true);
        double h = req("launcher.exit_height_m"), fwd = opt("launcher.offset_forward_m", 0), left = opt("launcher.offset_left_m", 0);
        double hMin = req("launcher.hood_min_deg"), hMax = req("launcher.hood_max_deg");
        boolean turret = flag("launcher.has_turret", true);
        Map<String, Launcher.BallFit> balls = new LinkedHashMap<>();
        for (String b : Catalog.BALLS) {
            Projectile.Ball ball = ball(b);
            double ratio = req("launcher." + b + ".exit_speed_ratio");
            double loss = opt("launcher." + b + ".energy_loss_factor", 1.6);
            if (ball != null && !Double.isNaN(ratio)) balls.put(b, new Launcher.BallFit(ball, ratio, loss));
        }
        if (anyNaN(t, is, w, i0, dia, moi, count, gear, kp, maxRpm, tol, feed, h, hMin, hMax) || balls.size() < Catalog.BALLS.size()) return null;
        MotorSpec motor = MotorSpec.fromRpm(t, is, w, i0, v);
        Flywheel.Params fp = new Flywheel.Params(moi, dia / 2, gear, (int) Math.round(count), motor, friction, kp, comp);
        return new Launcher.Params(fp, h, fwd, left, hMin, hMax, maxRpm, tol, feed, waitReady, turret, balls);
    }

    public Turret turret() {
        double v = opt("turret.max_speed_dps", Double.NaN), a = opt("turret.max_accel_dps2", Double.NaN);
        if (anyNaN(v, a)) return null;
        return new Turret(v, a, opt("turret.settle_time_s", 0), opt("turret.range_deg", 360));
    }

    /**
     * The upward-facing HIVE CELL opening, centered above the origin and facing +x (the launching
     * robot stands on the +x side).
     */
    public Aperture cell() {
        double w = req("target.cell.opening_width_m"), h = req("target.cell.opening_height_m");
        double z = req("target.cell.opening_center_height_m"), tilt = req("target.cell.opening_tilt_deg");
        if (anyNaN(w, h, z, tilt)) return null;
        return Aperture.oriented("HIVE CELL", new Vec3(0, 0, z), 0, Physics.rad(tilt), w, h, false);
    }

    public OdometryErrorModel.Params odometry() {
        double scale = req("localization.scale_error_std"), slip = req("localization.slip_std_per_sqrt_m");
        double bias = req("localization.gyro_bias_std_dps");
        double noise = opt("localization.gyro_noise_deg_per_sqrt_s", 0), gscale = opt("localization.gyro_scale_std", 0);
        double vis = opt("localization.vision_std_m", Double.NaN), visT = opt("localization.vision_interval_s", Double.NaN);
        if (anyNaN(scale, slip, bias)) return null;
        return new OdometryErrorModel.Params(scale, slip, bias, noise, gscale, vis, Double.isNaN(visT) ? 0 : visT);
    }

    private static boolean anyNaN(double... v) {
        for (double x : v) if (Double.isNaN(x)) return true;
        return false;
    }
}
