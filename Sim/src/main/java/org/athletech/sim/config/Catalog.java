package org.athletech.sim.config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Every piece of information the robot profile is meant to hold, with units and meaning. This is
 * the single source of truth: {@code profile init} creates one empty row per entry, {@code
 * profile docs} turns it into PROFILE.md, and the sims read and write these names.
 *
 * <p>Naming: {@code category.thing_unit}. The suffix is the unit ({@code _m}, {@code _s},
 * {@code _rpm}, {@code _deg}, {@code _a}, {@code _v}, {@code _ohm}, {@code _kg}, ...); no suffix
 * means a plain number (ratio, count, fraction, 0/1 flag).
 *
 * <p>To add something new: add it here (keep the naming), then run {@code profile init} on the
 * existing database - new rows are added, existing values are kept.
 */
public final class Catalog {
    private Catalog() {
    }

    /** What kind of information an entry is. */
    public enum Role {
        /** A property you put in (robot build, game element, error model). Sims read these. */
        INPUT,
        /** Computed by the sims from the inputs. */
        DERIVED,
        /** Comes from data logged on the real robot. */
        MEASURED;

        public String id() {
            return name().toLowerCase(java.util.Locale.ROOT);
        }
    }

    public static final class Param {
        private final String key;
        private final String unit;
        private final Role role;
        private final boolean required;
        private final String description;
        private final double min;
        private final double max;
        private final Double example;
        private final SourceKind exampleKind;

        public Param(String key, String unit, Role role, boolean required, String description, double min, double max, Double example, SourceKind exampleKind) {
            this.key = key;
            this.unit = unit;
            this.role = role;
            this.required = required;
            this.description = description;
            this.min = min;
            this.max = max;
            this.example = example;
            this.exampleKind = exampleKind;
        }

        public String key() { return key; }
        public String unit() { return unit; }
        public Role role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public double min() { return min; }
        public double max() { return max; }
        public Double example() { return example; }
        public SourceKind exampleKind() { return exampleKind; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Param)) return false;
            Param o = (Param) other;
            return java.util.Objects.equals(key, o.key)
                    && java.util.Objects.equals(unit, o.unit)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && Double.compare(min, o.min) == 0
                    && Double.compare(max, o.max) == 0
                    && java.util.Objects.equals(example, o.example)
                    && java.util.Objects.equals(exampleKind, o.exampleKind);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(key, unit, role, required, description, min, max, example, exampleKind);
        }

        @Override
        public String toString() {
            return "Param[key=" + key + ", unit=" + unit + ", role=" + role + ", required=" + required + ", description=" + description + ", min=" + min + ", max=" + max + ", example=" + example + ", exampleKind=" + exampleKind + "]";
        }

        public String category() { return Catalog.category(key); }
    
    }

    public static final class Curve {
        private final String name;
        private final String xName;
        private final String xUnit;
        private final String yName;
        private final String yUnit;
        private final Role role;
        private final boolean required;
        private final String description;

        public Curve(String name, String xName, String xUnit, String yName, String yUnit, Role role, boolean required, String description) {
            this.name = name;
            this.xName = xName;
            this.xUnit = xUnit;
            this.yName = yName;
            this.yUnit = yUnit;
            this.role = role;
            this.required = required;
            this.description = description;
        }

        public String name() { return name; }
        public String xName() { return xName; }
        public String xUnit() { return xUnit; }
        public String yName() { return yName; }
        public String yUnit() { return yUnit; }
        public Role role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Curve)) return false;
            Curve o = (Curve) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(xName, o.xName)
                    && java.util.Objects.equals(xUnit, o.xUnit)
                    && java.util.Objects.equals(yName, o.yName)
                    && java.util.Objects.equals(yUnit, o.yUnit)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, xName, xUnit, yName, yUnit, role, required, description);
        }

        @Override
        public String toString() {
            return "Curve[name=" + name + ", xName=" + xName + ", xUnit=" + xUnit + ", yName=" + yName + ", yUnit=" + yUnit + ", role=" + role + ", required=" + required + ", description=" + description + "]";
        }

        public String category() { return Catalog.category(name); }
    
    }

    public static final class Surface {
        private final String name;
        private final String xName;
        private final String xUnit;
        private final String yName;
        private final String yUnit;
        private final String zName;
        private final String zUnit;
        private final Role role;
        private final boolean required;
        private final String description;

        public Surface(String name, String xName, String xUnit, String yName, String yUnit, String zName, String zUnit, Role role, boolean required, String description) {
            this.name = name;
            this.xName = xName;
            this.xUnit = xUnit;
            this.yName = yName;
            this.yUnit = yUnit;
            this.zName = zName;
            this.zUnit = zUnit;
            this.role = role;
            this.required = required;
            this.description = description;
        }

        public String name() { return name; }
        public String xName() { return xName; }
        public String xUnit() { return xUnit; }
        public String yName() { return yName; }
        public String yUnit() { return yUnit; }
        public String zName() { return zName; }
        public String zUnit() { return zUnit; }
        public Role role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Surface)) return false;
            Surface o = (Surface) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(xName, o.xName)
                    && java.util.Objects.equals(xUnit, o.xUnit)
                    && java.util.Objects.equals(yName, o.yName)
                    && java.util.Objects.equals(yUnit, o.yUnit)
                    && java.util.Objects.equals(zName, o.zName)
                    && java.util.Objects.equals(zUnit, o.zUnit)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, xName, xUnit, yName, yUnit, zName, zUnit, role, required, description);
        }

        @Override
        public String toString() {
            return "Surface[name=" + name + ", xName=" + xName + ", xUnit=" + xUnit + ", yName=" + yName + ", yUnit=" + yUnit + ", zName=" + zName + ", zUnit=" + zUnit + ", role=" + role + ", required=" + required + ", description=" + description + "]";
        }

        public String category() { return Catalog.category(name); }
    
    }

    public static final class Dist {
        private final String name;
        private final String unit;
        private final String per;
        private final Role role;
        private final boolean required;
        private final String description;
        private final String exampleKind;
        private final Double exampleMean;
        private final Double exampleStd;

        public Dist(String name, String unit, String per, Role role, boolean required, String description, String exampleKind, Double exampleMean, Double exampleStd) {
            this.name = name;
            this.unit = unit;
            this.per = per;
            this.role = role;
            this.required = required;
            this.description = description;
            this.exampleKind = exampleKind;
            this.exampleMean = exampleMean;
            this.exampleStd = exampleStd;
        }

        public String name() { return name; }
        public String unit() { return unit; }
        public String per() { return per; }
        public Role role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public String exampleKind() { return exampleKind; }
        public Double exampleMean() { return exampleMean; }
        public Double exampleStd() { return exampleStd; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Dist)) return false;
            Dist o = (Dist) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(unit, o.unit)
                    && java.util.Objects.equals(per, o.per)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(exampleKind, o.exampleKind)
                    && java.util.Objects.equals(exampleMean, o.exampleMean)
                    && java.util.Objects.equals(exampleStd, o.exampleStd);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, unit, per, role, required, description, exampleKind, exampleMean, exampleStd);
        }

        @Override
        public String toString() {
            return "Dist[name=" + name + ", unit=" + unit + ", per=" + per + ", role=" + role + ", required=" + required + ", description=" + description + ", exampleKind=" + exampleKind + ", exampleMean=" + exampleMean + ", exampleStd=" + exampleStd + "]";
        }

        public String category() { return Catalog.category(name); }
    
    }

    public static final class Action {
        private final String name;
        private final Role role;
        private final boolean required;
        private final String description;
        private final Double exampleMeanS;
        private final Double exampleStdS;
        private final Double exampleSuccess;

        public Action(String name, Role role, boolean required, String description, Double exampleMeanS, Double exampleStdS, Double exampleSuccess) {
            this.name = name;
            this.role = role;
            this.required = required;
            this.description = description;
            this.exampleMeanS = exampleMeanS;
            this.exampleStdS = exampleStdS;
            this.exampleSuccess = exampleSuccess;
        }

        public String name() { return name; }
        public Role role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public Double exampleMeanS() { return exampleMeanS; }
        public Double exampleStdS() { return exampleStdS; }
        public Double exampleSuccess() { return exampleSuccess; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Action)) return false;
            Action o = (Action) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(exampleMeanS, o.exampleMeanS)
                    && java.util.Objects.equals(exampleStdS, o.exampleStdS)
                    && java.util.Objects.equals(exampleSuccess, o.exampleSuccess);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, role, required, description, exampleMeanS, exampleStdS, exampleSuccess);
        }

        @Override
        public String toString() {
            return "Action[name=" + name + ", role=" + role + ", required=" + required + ", description=" + description + ", exampleMeanS=" + exampleMeanS + ", exampleStdS=" + exampleStdS + ", exampleSuccess=" + exampleSuccess + "]";
        }

        public String category() { return Catalog.category(name); }
    
    }

    public static String category(String name) {
        int dot = name.indexOf('.');
        return dot < 0 ? name : name.substring(0, dot);
    }

    /** Game elements that can be launched (BIOBUZZ). */
    public static final List<String> BALLS = Arrays.asList("pollen", "nectar");

    public static final List<Param> PARAMS;
    public static final List<Curve> CURVES;
    public static final List<Surface> SURFACES;
    public static final List<Dist> DISTS;
    public static final List<Action> ACTIONS;

    /** Short explanation of each category, used in reports and PROFILE.md. */
    public static final List<String[]> CATEGORIES = Arrays.asList(
            new String[]{"robot", "Whole-robot properties: mass, size, capacity."},
            new String[]{"battery", "Battery and wiring: resting voltage and how much it sags per amp."},
            new String[]{"power", "Current draw of the whole robot (measured)."},
            new String[]{"drive", "Mecanum drivetrain build, motors, traction, and the resulting speed/acceleration."},
            new String[]{"launcher", "Flywheel launcher: build, controller, per-ball behavior, and shot results."},
            new String[]{"turret", "Turret slew speed and accuracy."},
            new String[]{"ball", "Game elements (POLLEN, NECTAR): size, mass, aerodynamics."},
            new String[]{"target", "Scoring openings (HIVE CELL, FLOWER) geometry."},
            new String[]{"hive", "HIVE structure behavior."},
            new String[]{"field", "Field dimensions and coordinates."},
            new String[]{"match", "Match timing."},
            new String[]{"game", "Game element counts."},
            new String[]{"localization", "How the robot's believed position drifts from the truth."},
            new String[]{"control", "Control loop timing."},
            new String[]{"driver", "Human driver behavior."},
            new String[]{"intake", "Collecting game elements."},
            new String[]{"flower", "Placing elements in FLOWERS."},
            new String[]{"garden", "Placing elements in GARDENS."},
            new String[]{"auto", "Autonomous-period actions."},
            new String[]{"park", "End-of-match parking."});

    private static final double NA = Double.NaN;

    private static Param in(String key, String unit, boolean required, String desc, double min, double max, Double example) {
        return new Param(key, unit, Role.INPUT, required, desc, min, max, example, SourceKind.PLACEHOLDER);
    }

    private static Param manual(String key, String unit, String desc, double value) {
        return new Param(key, unit, Role.INPUT, true, desc, NA, NA, value, SourceKind.GAME_MANUAL);
    }

    private static Param der(String key, String unit, boolean required, String desc) {
        return new Param(key, unit, Role.DERIVED, required, desc, NA, NA, null, null);
    }

    private static Param meas(String key, String unit, boolean required, String desc, double min, double max, Double example) {
        return new Param(key, unit, Role.MEASURED, required, desc, min, max, example, SourceKind.PLACEHOLDER);
    }

    private static Curve curve(String name, String x, String xu, String y, String yu, Role role, boolean req, String desc) {
        return new Curve(name, x, xu, y, yu, role, req, desc);
    }

    static {
        List<Param> p = new ArrayList<>();
        // ---------------------------------------------------------------- robot
        p.add(in("robot.mass_kg", "kg", true, "Total robot mass ready to play, battery included.", 2, 25, 13.0));
        p.add(in("robot.length_m", "m", true, "Frame length front to back.", 0.1, 0.6, 0.43));
        p.add(in("robot.width_m", "m", true, "Frame width side to side.", 0.1, 0.6, 0.43));
        p.add(in("robot.moi_kgm2", "kg*m^2", false, "Yaw moment of inertia about the robot center. Leave empty to estimate it as mass*(length^2+width^2)/12.", 0.01, 2, null));
        p.add(in("robot.max_pollen_held", "count", true, "POLLEN the robot can hold at once.", 0, 40, 4.0));
        p.add(in("robot.max_nectar_held", "count", true, "NECTAR the robot can hold at once.", 0, 16, 2.0));
        p.add(in("robot.electronics_current_a", "A", false, "Current drawn by hubs, sensors and idle servos when nothing moves.", 0, 5, 0.8));
        // ---------------------------------------------------------------- battery
        p.add(in("battery.open_circuit_v", "V", true, "Resting voltage of a freshly charged battery (no load). Sims use this as the starting voltage.", 10, 15, 13.2));
        p.add(meas("battery.internal_resistance_ohm", "ohm", true, "Battery + wiring + switch resistance: voltage drop per amp drawn. Fit from HardwareMonitor sessions (profile import-monitor).", 0.005, 0.5, 0.07));
        p.add(in("battery.capacity_mah", "mAh", false, "Rated battery capacity.", 500, 6000, 3000.0));
        p.add(in("battery.voltage_drop_per_ah", "V/Ah", false, "How far the resting voltage falls per amp-hour used.", 0, 5, 0.6));
        p.add(meas("battery.end_of_match_v", "V", false, "Typical resting voltage after a full match.", 9, 14, null));
        // ---------------------------------------------------------------- power (measured)
        p.add(meas("power.avg_current_a", "A", false, "Average total current over a match.", 0, 60, null));
        p.add(meas("power.peak_current_a", "A", false, "Highest total current seen (95th percentile of samples).", 0, 80, null));
        p.add(meas("power.min_battery_v", "V", false, "Lowest battery voltage seen under load.", 5, 15, null));
        // ---------------------------------------------------------------- drive inputs
        p.add(in("drive.wheel_diameter_m", "m", true, "Mecanum wheel diameter.", 0.04, 0.2, 0.104));
        p.add(in("drive.track_width_m", "m", true, "Distance between left and right wheel centers.", 0.1, 0.5, 0.36));
        p.add(in("drive.wheelbase_m", "m", true, "Distance between front and back wheel centers.", 0.1, 0.5, 0.30));
        p.add(in("drive.gear_ratio", "ratio", true, "Extra reduction from the motor output shaft to the wheel (1 = wheel on the motor shaft, 2 = wheel turns half as fast).", 0.1, 20, 1.0));
        p.add(in("drive.motor_free_speed_rpm", "rpm", true, "Drive motor free speed at its output shaft (gearbox included) at nominal voltage, from the datasheet.", 10, 10000, 312.0));
        p.add(in("drive.motor_stall_torque_nm", "N*m", true, "Drive motor stall torque at its output shaft at nominal voltage (kg*cm x 0.0981).", 0.01, 50, 2.383));
        p.add(in("drive.motor_stall_current_a", "A", true, "Drive motor stall current at nominal voltage.", 0.5, 50, 9.2));
        p.add(in("drive.motor_free_current_a", "A", true, "Drive motor free (no-load) current at nominal voltage.", 0, 5, 0.25));
        p.add(in("drive.motor_nominal_v", "V", false, "Voltage the motor datasheet is quoted at.", 6, 14, 12.0));
        p.add(in("drive.wheel_friction_coeff", "mu", true, "Wheel-to-tile friction coefficient. Limits acceleration (traction).", 0.1, 2, 0.9));
        p.add(in("drive.strafe_efficiency", "fraction", true, "Strafing speed and force relative to driving forward (mecanum roller losses), 0-1.", 0.3, 1, 0.8));
        p.add(in("drive.rolling_resistance_coeff", "fraction", false, "Rolling + scrub resistance as a fraction of robot weight.", 0, 0.3, 0.04));
        p.add(in("drive.current_limit_a", "A", false, "Per-motor current limit (software or hub). Empty = no limit.", 1, 50, null));
        p.add(in("drive.max_power", "fraction", false, "Software cap on drive motor power (1 = full).", 0.1, 1, 1.0));
        p.add(in("drive.zero_power_brake", "flag", false, "1 = motors BRAKE at zero power, 0 = FLOAT.", 0, 1, 1.0));
        p.add(in("drive.encoder_ticks_per_rev", "ticks", false, "Encoder counts per motor output-shaft revolution (needed to turn HardwareMonitor velocity into m/s).", 1, 100000, 537.7));
        p.add(new Param("drive.motor_names", "text", Role.INPUT, false,
                "Comma-separated drive motor names in the robot configuration (used by profile import-monitor), e.g. leftFront,rightFront,leftBack,rightBack.",
                NA, NA, null, null));
        p.add(in("drive.path_overhead_factor", "ratio", true, "Real path-following time divided by the physics-limited time from the sims (>= 1). Measure: time a real path / sim prediction.", 1, 5, 1.3));
        // ---------------------------------------------------------------- drive derived / measured
        p.add(der("drive.max_speed_mps", "m/s", true, "Top forward speed on a full battery."));
        p.add(der("drive.max_strafe_speed_mps", "m/s", true, "Top sideways speed on a full battery."));
        p.add(der("drive.max_turn_rate_dps", "deg/s", true, "Top turning rate on a full battery."));
        p.add(der("drive.max_accel_mps2", "m/s^2", true, "Peak forward acceleration from rest (traction or motor limited)."));
        p.add(der("drive.max_decel_mps2", "m/s^2", true, "Peak deceleration when reversing power at top speed."));
        p.add(der("drive.time_to_max_speed_s", "s", false, "Time from rest to 95% of top speed at full power."));
        p.add(der("drive.peak_current_a", "A", false, "Peak total drive current (all 4 motors) accelerating from rest."));
        p.add(der("drive.cruise_current_a", "A", false, "Total drive current holding top speed."));
        p.add(meas("drive.measured_top_speed_mps", "m/s", false, "Highest drive wheel speed seen in HardwareMonitor sessions.", 0, 10, null));
        // ---------------------------------------------------------------- launcher inputs
        p.add(in("launcher.flywheel_diameter_m", "m", true, "Flywheel diameter (contact surface).", 0.02, 0.3, 0.096));
        p.add(in("launcher.flywheel_moi_kgm2", "kg*m^2", true, "Rotational inertia of everything spinning with the flywheel, at flywheel speed (CAD gives this).", 1e-5, 0.1, 0.0012));
        p.add(in("launcher.motor_count", "count", true, "Motors driving the flywheel.", 1, 4, 2.0));
        p.add(in("launcher.gear_ratio", "ratio", true, "Motor shaft speed / flywheel speed (0.5 = flywheel spins twice as fast as the motor).", 0.1, 10, 1.0));
        p.add(in("launcher.motor_free_speed_rpm", "rpm", true, "Launcher motor free speed at its output shaft at nominal voltage.", 100, 30000, 6000.0));
        p.add(in("launcher.motor_stall_torque_nm", "N*m", true, "Launcher motor stall torque at its output shaft.", 0.01, 10, 0.144));
        p.add(in("launcher.motor_stall_current_a", "A", true, "Launcher motor stall current.", 0.5, 50, 9.2));
        p.add(in("launcher.motor_free_current_a", "A", true, "Launcher motor free current.", 0, 5, 0.25));
        p.add(in("launcher.motor_nominal_v", "V", false, "Voltage the launcher motor datasheet is quoted at.", 6, 14, 12.0));
        p.add(in("launcher.friction_torque_nm", "N*m", false, "Extra drag at the flywheel (belts, bearings) beyond the motors' own free current.", 0, 1, 0.005));
        p.add(in("launcher.kp_per_rpm", "duty/rpm", true, "Velocity controller proportional gain: power added per rpm below target (feedforward is computed from the motor model).", 0, 0.1, 0.002));
        p.add(in("launcher.voltage_compensation", "flag", false, "1 = feedforward scaled by 12 V / battery voltage, 0 = fixed.", 0, 1, 0.0));
        p.add(in("launcher.max_rpm", "rpm", true, "Highest flywheel speed the code will command.", 100, 30000, 4500.0));
        p.add(in("launcher.ready_tolerance_rpm", "rpm", true, "The code feeds a ball only when the flywheel is within this many rpm of target.", 1, 2000, 100.0));
        p.add(in("launcher.wait_for_ready", "flag", false, "1 = wait for the flywheel to be within tolerance before each feed, 0 = feed on a fixed interval.", 0, 1, 1.0));
        p.add(in("launcher.feed_interval_s", "s", true, "Minimum time between consecutive balls in a burst (feeder speed).", 0.02, 3, 0.25));
        p.add(in("launcher.exit_height_m", "m", true, "Height of the ball center where it leaves the launcher.", 0.05, 1.5, 0.35));
        p.add(in("launcher.offset_forward_m", "m", false, "Launch point ahead of the robot center.", -0.5, 0.5, 0.0));
        p.add(in("launcher.offset_left_m", "m", false, "Launch point left of the robot center.", -0.5, 0.5, 0.0));
        p.add(in("launcher.hood_min_deg", "deg", true, "Lowest launch angle above horizontal the hood can set.", 0, 90, 30.0));
        p.add(in("launcher.hood_max_deg", "deg", true, "Highest launch angle above horizontal.", 0, 90, 65.0));
        p.add(in("launcher.hood_speed_dps", "deg/s", false, "How fast the hood moves.", 1, 2000, 300.0));
        p.add(in("launcher.has_turret", "flag", false, "1 = launcher aims with a turret, 0 = robot turns to aim.", 0, 1, 1.0));
        for (String b : BALLS) {
            String B = b.toUpperCase();
            p.add(in("launcher." + b + ".exit_speed_ratio", "fraction", true, B + " exit speed / flywheel surface speed. Measure with slow-motion video.", 0.05, 1, b.equals("pollen") ? 0.5 : 0.45));
            p.add(in("launcher." + b + ".energy_loss_factor", "ratio", false, "Flywheel energy taken per " + B + " shot / ball kinetic energy (>= 1; spin, squish and slip).", 1, 5, 1.6));
        }
        // ---------------------------------------------------------------- launcher derived
        p.add(der("launcher.spinup_time_s", "s", true, "Time from a stopped flywheel to max_rpm within tolerance, full battery."));
        p.add(der("launcher.hold_current_a", "A", false, "Battery current to hold the flywheel at max_rpm (for battery-drain models)."));
        for (String b : BALLS) {
            String B = b.toUpperCase();
            p.add(der("launcher." + b + ".rpm_drop_per_shot_rpm", "rpm", false, "Flywheel speed lost firing one " + B + " at max_rpm."));
            p.add(der("launcher." + b + ".recovery_time_s", "s", true, "Time to win back 90% of the speed lost firing one " + B + " at max_rpm, full battery."));
            p.add(der("launcher." + b + ".max_range_m", "m", false, "Farthest distance from which a " + B + " shot can still score in the HIVE CELL (searched up to the field diagonal)."));
        }
        // ---------------------------------------------------------------- turret
        p.add(in("turret.max_speed_dps", "deg/s", false, "Turret top slew speed.", 1, 3000, 360.0));
        p.add(in("turret.max_accel_dps2", "deg/s^2", false, "Turret acceleration.", 1, 50000, 1800.0));
        p.add(in("turret.settle_time_s", "s", false, "Extra time to settle on target after a move.", 0, 2, 0.08));
        p.add(in("turret.range_deg", "deg", false, "Total turret travel.", 0, 720, 270.0));
        // ---------------------------------------------------------------- balls (game elements)
        p.add(manual("ball.pollen.diameter_m", "m", "POLLEN diameter, about 2.8 in (Competition Manual 9.8). Balls vary.", 0.071));
        p.add(manual("ball.nectar.diameter_m", "m", "NECTAR diameter, about 3.6 in (Competition Manual 9.8). Balls vary.", 0.091));
        for (String b : BALLS) {
            String B = b.toUpperCase();
            p.add(in("ball." + b + ".mass_kg", "kg", true, B + " mass. Not in the manual: weigh a few on a kitchen scale.", 0.001, 0.5, b.equals("pollen") ? 0.02 : 0.035));
            p.add(in("ball." + b + ".drag_coeff", "Cd", false, B + " drag coefficient (smooth ball about 0.5; holes/texture raise it).", 0.1, 2, 0.5));
            p.add(in("ball." + b + ".lift_coeff", "Cl", false, B + " Magnus lift coefficient from spin: + = backspin (floats), - = topspin (dips), 0 = none.", -1, 1, 0.0));
        }
        // ---------------------------------------------------------------- targets
        p.add(manual("target.cell.opening_width_m", "m", "HIVE CELL opening width, about 20 in (9.6.2).", 0.508));
        p.add(manual("target.cell.opening_height_m", "m", "HIVE CELL opening height, about 14 in (9.6.2).", 0.356));
        p.add(manual("target.cell.depth_m", "m", "HIVE CELL depth, about 12 in (9.6.2).", 0.305));
        p.add(manual("target.cell.spacing_m", "m", "Distance between the two CELLS on a HIVE, about 18.8 in (9.6.2).", 0.478));
        p.add(in("target.cell.opening_center_height_m", "m", true, "Height above the tiles of the center of the upward-facing CELL's opening. Not in the manual text: measure the field.", 0.3, 2.5, 1.25));
        p.add(in("target.cell.opening_tilt_deg", "deg", true, "How far the upward-facing CELL's opening tilts from facing straight up, toward the launching robot (0 = straight up, 90 = vertical).", 0, 90, 20.0));
        p.add(manual("target.flower.opening_diameter_m", "m", "FLOWER top opening, about 4 in (9.7).", 0.1015));
        p.add(manual("target.flower.opening_height_m", "m", "FLOWER top opening height above the tiles, about 21.5 in (9.7).", 0.546));
        // ---------------------------------------------------------------- hive / field / match / game
        p.add(manual("hive.pivot_height_m", "m", "HIVE pivot axis height above the tiles, 43.95 in (9.6.1).", 1.1165));
        p.add(manual("hive.frame_width_m", "m", "HIVE frame width, 49.46 in (9.6.1).", 1.2565));
        p.add(in("hive.pollen_to_tip", "count", true, "POLLEN that must land in the upward CELL before the HIVE tips (measure on a real HIVE).", 1, 40, 4.0));
        p.add(in("hive.nectar_to_tip", "count", true, "NECTAR that must land in the upward CELL before the HIVE tips.", 1, 16, 2.0));
        p.add(manual("field.size_m", "m", "Inside of the perimeter, 144 in square (9.2).", 3.6575));
        p.add(manual("field.tile_size_m", "m", "Foam tile size, 24 in (Competition Manual section 9).", 0.6095));
        p.add(manual("match.auto_s", "s", "AUTO period (10).", 30));
        p.add(manual("match.transition_s", "s", "Pause between AUTO and TELEOP (10).", 8));
        p.add(manual("match.teleop_s", "s", "TELEOP period (10).", 120));
        p.add(manual("match.flower_window_s", "s", "FLOWER scoring opens with this much time left (10).", 60));
        p.add(manual("game.pollen_total", "count", "POLLEN in a match (10).", 40));
        p.add(manual("game.nectar_per_alliance", "count", "NECTAR per alliance color (10).", 8));
        p.add(manual("game.preload_pollen_per_robot", "count", "POLLEN each robot starts with (10).", 4));
        // ---------------------------------------------------------------- localization
        p.add(new Param("localization.method", "text", Role.INPUT, false,
                "How the robot localizes, e.g. 'goBILDA Pinpoint, 2 pods + IMU' or 'drive encoders + IMU'.", NA, NA, null, null));
        p.add(in("localization.scale_error_std", "fraction", true, "1-sigma distance calibration error of the odometry (0.005 = 0.5%).", 0, 0.2, 0.005));
        p.add(in("localization.slip_std_per_sqrt_m", "m/sqrt(m)", true, "Random position error growth: 1-sigma error after 1 m of travel from slip and bumps.", 0, 0.2, 0.008));
        p.add(in("localization.gyro_bias_std_dps", "deg/s", true, "1-sigma gyro bias left after calibration.", 0, 1, 0.005));
        p.add(in("localization.gyro_noise_deg_per_sqrt_s", "deg/sqrt(s)", false, "Gyro random walk.", 0, 1, 0.02));
        p.add(in("localization.gyro_scale_std", "fraction", false, "1-sigma gyro scale error (heading error per degree turned).", 0, 0.1, 0.002));
        p.add(in("localization.vision_std_m", "m", false, "1-sigma position error of an AprilTag relocalization. Empty = no vision correction.", 0, 1, null));
        p.add(in("localization.vision_interval_s", "s", false, "Typical time between vision relocalizations.", 0.1, 200, null));
        p.add(der("localization.position_error_after_match_m", "m", false, "RMS position error at the end of a full match of driving."));
        p.add(der("localization.heading_error_after_match_deg", "deg", false, "RMS heading error at the end of a full match."));
        // ---------------------------------------------------------------- control
        p.add(meas("control.loop_time_ms", "ms", true, "Average control loop time (HardwareMonitor).", 0.5, 200, null));
        p.add(meas("control.loop_time_p95_ms", "ms", false, "95th percentile loop time (HardwareMonitor).", 0.5, 500, null));
        PARAMS = Collections.unmodifiableList(p);

        List<Curve> c = new ArrayList<>();
        c.add(curve("drive.speed_vs_time", "time", "s", "forward speed", "m/s", Role.DERIVED, false, "Speed from rest at full power, full battery."));
        c.add(curve("drive.current_vs_time", "time", "s", "drive current", "A", Role.DERIVED, false, "Total drive motor current from rest at full power."));
        c.add(curve("drive.current_vs_speed", "forward speed", "m/s", "drive current", "A", Role.DERIVED, true, "Total drive current at full power vs speed (current falls as the robot speeds up)."));
        c.add(curve("drive.time_to_distance", "distance", "m", "time", "s", Role.DERIVED, true, "Fastest stop-to-stop time driving forward (accelerate, then brake). Physics limit: multiply by drive.path_overhead_factor for real paths."));
        c.add(curve("drive.strafe_time_to_distance", "distance", "m", "time", "s", Role.DERIVED, true, "Fastest stop-to-stop time strafing sideways."));
        c.add(curve("drive.time_to_turn", "angle", "deg", "time", "s", Role.DERIVED, true, "Fastest stop-to-stop time turning in place."));
        c.add(curve("drive.top_speed_vs_battery_v", "battery resting voltage", "V", "top speed", "m/s", Role.DERIVED, true, "How top speed falls as the battery drains."));
        c.add(curve("drive.max_accel_vs_battery_v", "battery resting voltage", "V", "peak acceleration", "m/s^2", Role.DERIVED, true, "How peak acceleration falls as the battery drains."));
        c.add(curve("battery.voltage_vs_current", "total current", "A", "battery voltage", "V", Role.MEASURED, true, "Battery terminal voltage vs total current (sag), binned from HardwareMonitor sessions."));
        c.add(curve("battery.voltage_vs_match_time", "match time", "s", "battery voltage", "V", Role.MEASURED, false, "Battery voltage through a logged session."));
        c.add(curve("launcher.rpm_vs_time", "time", "s", "flywheel speed", "rpm", Role.DERIVED, false, "Spin-up from stopped to max_rpm, full battery."));
        c.add(curve("launcher.spinup_time_vs_battery_v", "battery resting voltage", "V", "spin-up time", "s", Role.DERIVED, false, "Spin-up time to max_rpm vs battery."));
        for (String b : BALLS) {
            String B = b.toUpperCase();
            c.add(curve("launcher." + b + ".exit_speed_vs_rpm", "flywheel speed", "rpm", "exit speed", "m/s", Role.DERIVED, false, B + " exit speed vs flywheel speed."));
            c.add(curve("launcher." + b + ".target_rpm_vs_distance", "distance to CELL", "m", "flywheel speed", "rpm", Role.DERIVED, true, "Flywheel speed to command for a " + B + " shot into the upward HIVE CELL (the shooting lookup table)."));
            c.add(curve("launcher." + b + ".hood_vs_distance", "distance to CELL", "m", "launch angle", "deg", Role.DERIVED, true, "Hood angle to use at each distance for " + B + "."));
            c.add(curve("launcher." + b + ".make_prob_vs_distance", "distance to CELL", "m", "make probability", "fraction", Role.DERIVED, true, "Chance one " + B + " shot scores from a rested flywheel. y_std = standard error."));
            c.add(curve("launcher." + b + ".burst_make_prob_vs_distance", "distance to CELL", "m", "make probability", "fraction", Role.DERIVED, true, "Average chance per ball when firing a full load of " + B + " back to back."));
            c.add(curve("launcher." + b + ".measured_make_prob_vs_distance", "distance to CELL", "m", "make probability", "fraction", Role.MEASURED, false,
                    "Make rate of logged real single " + B + " shots, 0.25 m bins (n = shots per bin). Prefer over the sim curve where n is large."));
            c.add(curve("launcher." + b + ".measured_burst_make_prob_vs_distance", "distance to CELL", "m", "make probability", "fraction", Role.MEASURED, false,
                    "Make rate of logged real " + B + " shots fired in bursts, 0.25 m bins."));
            c.add(curve("launcher." + b + ".recovery_time_vs_battery_v", "battery resting voltage", "V", "recovery time", "s", Role.DERIVED, false, "Flywheel recovery after one " + B + " shot vs battery."));
            c.add(curve("launcher." + b + ".burst_time_vs_battery_v", "battery resting voltage", "V", "burst time", "s", Role.DERIVED, true, "Time to fire a full load of " + B + " (first ball to last) vs battery, from a flywheel already at speed."));
        }
        c.add(curve("turret.time_to_turn", "angle", "deg", "time", "s", Role.DERIVED, false, "Turret move + settle time."));
        c.add(curve("localization.position_error_vs_distance", "distance driven", "m", "position error (RMS)", "m", Role.DERIVED, true, "How far off the believed position is after driving a distance (no vision fixes unless configured)."));
        c.add(curve("localization.position_error_vs_time", "time", "s", "position error (RMS)", "m", Role.DERIVED, true, "Position error through a match of typical driving."));
        c.add(curve("localization.heading_error_vs_time", "time", "s", "heading error (RMS)", "deg", Role.DERIVED, true, "Heading error through a match of typical driving."));
        c.add(curve("localization.measured_position_error_vs_distance", "distance driven", "m", "position error (RMS)", "m", Role.MEASURED, false, "From logged real pose checks (profile import-poses), 0.5 m bins."));
        c.add(curve("localization.measured_position_error_vs_time", "time", "s", "position error (RMS)", "m", Role.MEASURED, false, "From logged real pose checks, 10 s bins."));
        CURVES = Collections.unmodifiableList(c);

        List<Surface> s = new ArrayList<>();
        s.add(new Surface("drive.time_to_distance_vs_battery", "distance", "m", "battery resting voltage", "V", "time", "s", Role.DERIVED, false, "Stop-to-stop forward time by distance and battery."));
        for (String b : BALLS) {
            String B = b.toUpperCase();
            s.add(new Surface("launcher." + b + ".burst_make_prob_vs_distance_battery", "distance to CELL", "m", "battery resting voltage", "V",
                    "make probability", "fraction", Role.DERIVED, true, "Average chance per " + B + " in a full burst, by distance and battery (low battery = slower recovery = weaker later shots)."));
        }
        SURFACES = Collections.unmodifiableList(s);

        List<Dist> d = new ArrayList<>();
        d.add(new Dist("launcher.rpm_error", "rpm", "shot", Role.INPUT, true, "Flywheel speed error when a ball is fed (actual - target), beyond what the sim already models.", "normal", 0.0, 40.0));
        d.add(new Dist("launcher.hood_error_deg", "deg", "shot", Role.INPUT, false, "Hood angle error at the shot.", "normal", 0.0, 0.5));
        d.add(new Dist("launcher.exit_speed_variation", "fraction", "shot", Role.INPUT, true, "Ball-to-ball exit speed scatter as a fraction (ball size, wear).", "normal", 0.0, 0.02));
        d.add(new Dist("launcher.exit_angle_variation_deg", "deg", "shot", Role.INPUT, false, "Ball-to-ball launch angle scatter.", "normal", 0.0, 1.0));
        d.add(new Dist("turret.aim_error_deg", "deg", "shot", Role.INPUT, true, "Yaw aim error at the moment of the shot.", "normal", 0.0, 1.0));
        d.add(new Dist("localization.pose_error_m", "m", "shot", Role.DERIVED, true, "Error of the robot's believed position when it shoots, per axis (from the localization sim).", null, null, null));
        d.add(new Dist("localization.heading_error_deg", "deg", "shot", Role.DERIVED, false, "Error of the robot's believed heading when it shoots (from the localization sim); turns into aim error.", null, null, null));
        d.add(new Dist("drive.path_end_error_m", "m", "move", Role.INPUT, false, "Distance between where a path should end and where the robot stops.", "normal", 0.0, 0.03));
        d.add(new Dist("control.loop_time_ms", "ms", "loop", Role.MEASURED, true, "Loop time distribution from HardwareMonitor (empirical samples).", null, null, null));
        d.add(new Dist("driver.reaction_time_s", "s", "decision", Role.INPUT, false, "Driver reaction / decision delay in TELEOP.", "lognormal", 0.35, 0.15));
        DISTS = Collections.unmodifiableList(d);

        List<Action> a = new ArrayList<>();
        a.add(new Action("intake.pollen_floor", Role.INPUT, true, "Collect one POLLEN off the floor once the robot is lined up with it.", 0.6, 0.25, 0.9));
        a.add(new Action("intake.pollen_flower_bottom", Role.INPUT, false, "Pull one POLLEN out of a FLOWER's bottom retrieval opening.", 1.5, 0.5, 0.8));
        a.add(new Action("intake.nectar", Role.INPUT, true, "Collect one NECTAR once lined up with it.", 0.8, 0.3, 0.85));
        a.add(new Action("launcher.aim", Role.INPUT, true, "Settle aim (turret/drive + hood) once at the shooting spot.", 0.3, 0.1, 1.0));
        a.add(new Action("launcher.spinup_from_idle", Role.DERIVED, false, "Flywheel from stopped to shooting speed (from the launcher sim).", null, null, null));
        a.add(new Action("flower.place_nectar", Role.INPUT, true, "Place one NECTAR into the top of a FLOWER once lined up.", 1.5, 0.5, 0.85));
        a.add(new Action("flower.place_pollen", Role.INPUT, false, "Place one POLLEN into the top of a FLOWER once lined up.", 1.2, 0.4, 0.85));
        a.add(new Action("garden.place_pollen", Role.INPUT, false, "Drop one POLLEN into a GARDEN.", 0.5, 0.2, 0.95));
        a.add(new Action("auto.leave", Role.INPUT, false, "Get off the wall at the start of AUTO.", 0.5, 0.1, 0.99));
        a.add(new Action("park.loading_zone", Role.INPUT, false, "Final move into the LOADING ZONE (excluding the drive there).", 0.5, 0.2, 0.97));
        ACTIONS = Collections.unmodifiableList(a);
    }

    public static Param param(String key) {
        for (Param x : PARAMS) if (x.key().equals(key)) return x;
        return null;
    }

    public static Curve curve(String name) {
        for (Curve x : CURVES) if (x.name().equals(name)) return x;
        return null;
    }

    public static Surface surface(String name) {
        for (Surface x : SURFACES) if (x.name().equals(name)) return x;
        return null;
    }

    public static Dist dist(String name) {
        for (Dist x : DISTS) if (x.name().equals(name)) return x;
        return null;
    }

    public static Action action(String name) {
        for (Action x : ACTIONS) if (x.name().equals(name)) return x;
        return null;
    }

    /** Changes whenever any catalog entry changes; the DB only re-syncs when it differs. */
    public static String fingerprint() {
        StringBuilder sb = new StringBuilder();
        for (Object o : PARAMS) sb.append(o).append('\n');
        for (Object o : CURVES) sb.append(o).append('\n');
        for (Object o : SURFACES) sb.append(o).append('\n');
        for (Object o : DISTS) sb.append(o).append('\n');
        for (Object o : ACTIONS) sb.append(o).append('\n');
        try {
            byte[] h = java.security.MessageDigest.getInstance("SHA-256").digest(sb.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8));
            StringBuilder hex = new StringBuilder();
            for (int i = 0; i < 12; i++) hex.append(String.format("%02x", h[i] & 0xff));
            return hex.toString();
        } catch (java.security.NoSuchAlgorithmException e) {
            return Integer.toHexString(sb.toString().hashCode());
        }
    }

    public static String categoryDescription(String category) {
        for (String[] c : CATEGORIES) if (c[0].equals(category)) return c[1];
        return "";
    }
}
