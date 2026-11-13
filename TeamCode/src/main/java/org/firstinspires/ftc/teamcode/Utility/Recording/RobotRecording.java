package org.firstinspires.ftc.teamcode.Utility.Recording;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.DriveController;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Utility.AutoHandoff;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazePinpointLocalizer;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Date;

public final class RobotRecording {

    private RobotRecording() {}

    static final String[] CONSTANTS = {
            "BlazeConstants", "BlockerConstants", "DeviceConstants", "DrivetrainConstants", "FieldConstants", "FlywheelConstants",
            "GeneralConstants", "IntakeConstants", "ShooterConstants", "TunedConstants", "TurretConstants", "VisionConstants",
            "Autos.BlueParkConstants", "Autos.BluePreloadParkConstants", "Autos.BlueTwoTipGardenConstants",
            "Autos.RedParkConstants", "Autos.RedPreloadParkConstants", "Autos.RedTwoTipGardenConstants"
    };

    public static Recorder open(String opMode) {
        return new Recorder(Recorder.folderFor(opMode));
    }

    private static double tpd() {
        return ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    public static void loop(Recorder r) {

        final Runtime runtime = Runtime.getRuntime();
        r.channel("loop", "loop_ms", "ms", "last loop (HardwareMonitor)", () -> HardwareMonitor.get().getLastLoopMs());
        r.channel("loop", "loop_avg_ms", "ms", "average loop so far", () -> HardwareMonitor.get().getAverageLoopMs());
        r.channel("power", "battery_v", "V", "battery (sampled)", () -> HardwareMonitor.get().getBatteryVoltage());
        r.channel("power", "current_a", "A", "all hubs (sampled)", () -> HardwareMonitor.get().getTotalCurrentAmps());
        r.channel("power", "energy_wh", "Wh", "used so far", () -> HardwareMonitor.get().getEnergyWh());
        r.channel("loop", "heap_mb", "MB", "Java heap in use", () -> (runtime.totalMemory() - runtime.freeMemory()) / 1048576.0);
    }

    public static void blaze(Recorder r) {

        r.channel("blaze", "open", "0/1", "started this OpMode", () -> BlazeSession.isOpen() ? 1 : 0);
        r.channel("blaze", "running", "0/1", "", () -> BlazeSession.isRunning() ? 1 : 0);
        r.channel("blaze", "ch_age_ms", "ms", "since Blaze's last Control Hub bulk update (NaN = not fed)", () -> BlazeSession.bulkAgeMs(BlazeSession.CONTROL_HUB));
        r.channel("blaze", "eh_age_ms", "ms", "Expansion Hub", () -> BlazeSession.bulkAgeMs(BlazeSession.EXPANSION_HUB));
        r.channel("blaze", "ch_live", "0/1", "1 = this loop read Blaze's Control Hub data, 0 = stale so normal reads", () -> BlazeSession.fedNow(BlazeSession.CONTROL_HUB) ? 1 : 0);
        r.channel("blaze", "eh_live", "0/1", "", () -> BlazeSession.fedNow(BlazeSession.EXPANSION_HUB) ? 1 : 0);
        r.channel("blaze", "ch_updates", "", "Control Hub bulk updates so far", () -> BlazeSession.BULK[BlazeSession.CONTROL_HUB].total);
        r.channel("blaze", "eh_updates", "", "", () -> BlazeSession.BULK[BlazeSession.EXPANSION_HUB].total);
        r.channel("blaze", "pinpoint_updates", "", "Pinpoint samples from Blaze so far", () -> BlazeSession.PINPOINT.total);
        r.channel("blaze", "pinpoint_used", "", "loops that used Blaze's Pinpoint sample", () -> {

            BlazePinpointLocalizer p = BlazeSession.pinpoint();
            return p == null ? Double.NaN : p.samples;
        });
        r.channel("blaze", "pinpoint_sdk_reads", "", "loops that read the Pinpoint through the SDK instead", () -> {

            BlazePinpointLocalizer p = BlazeSession.pinpoint();
            return p == null ? Double.NaN : p.sdkReads;
        });
        r.watch("blaze", "fault", () -> BlazeSession.fault);
    }

    public static void gamepad(Recorder r, String name, final BetterGamepad g) {

        if (g == null) return;
        r.channel(name, "left_x", "", "stick", () -> g.left_stick_x());
        r.channel(name, "left_y", "", "stick (up is -)", () -> g.left_stick_y());
        r.channel(name, "right_x", "", "stick", () -> g.right_stick_x());
        r.channel(name, "right_y", "", "stick (up is -)", () -> g.right_stick_y());
        r.channel(name, "left_trigger", "", "0-1", () -> g.left_trigger());
        r.channel(name, "right_trigger", "", "0-1", () -> g.right_trigger());
        r.flag(name, "a", "", g::a);
        r.flag(name, "b", "", g::b);
        r.flag(name, "x", "", g::x);
        r.flag(name, "y", "", g::y);
        r.flag(name, "dpad_up", "", g::dpad_up);
        r.flag(name, "dpad_down", "", g::dpad_down);
        r.flag(name, "dpad_left", "", g::dpad_left);
        r.flag(name, "dpad_right", "", g::dpad_right);
        r.flag(name, "left_bumper", "", g::left_bumper);
        r.flag(name, "right_bumper", "", g::right_bumper);
        r.flag(name, "back", "", g::back);
        r.flag(name, "options", "start", g::options);
        r.flag(name, "share", "", g::share);
        r.flag(name, "main", "PS / guide", g::main_button);
        r.flag(name, "left_stick_button", "", g::left_stick_button);
        r.flag(name, "right_stick_button", "", g::right_stick_button);
    }

    private static final class Motion {
        double x = Double.NaN, y = Double.NaN, heading = Double.NaN, vx = Double.NaN, vy = Double.NaN, omega = Double.NaN;
    }

    public static void pose(Recorder r, final Follower follower) {

        if (follower == null) return;
        final Motion m = new Motion();
        r.prepare(() -> {
            Pose p = follower.pose();
            Velocity v = follower.velocity();
            m.x = p.x();
            m.y = p.y();
            m.heading = p.heading();
            m.vx = v.vx;
            m.vy = v.vy;
            m.omega = v.omega;
        });
        r.channel("pose", "x", "in", "field (FieldConstants frame)", () -> m.x);
        r.channel("pose", "y", "in", "field", () -> m.y);
        r.channel("pose", "heading", "deg", "field, unwrapped as Pedro gives it", () -> Math.toDegrees(m.heading));
        r.channel("pose", "vx", "in/s", "field", () -> m.vx);
        r.channel("pose", "vy", "in/s", "field", () -> m.vy);
        r.channel("pose", "speed", "in/s", "", () -> Math.hypot(m.vx, m.vy));
        r.channel("pose", "omega", "deg/s", "turn rate (+ = counterclockwise)", () -> Math.toDegrees(m.omega));
        r.channel("pose", "blaze_age_ms", "ms", "age of Blaze's Pinpoint sample (NaN = no Blaze)", () -> {

            BlazePinpointLocalizer p = BlazeSession.pinpoint();
            return p == null ? Double.NaN : 1000 * p.sampleAgeSeconds;
        });
        r.channel("pose", "blaze_sdk", "0/1", "1 = that sample was stale, read the SDK instead", () -> {

            BlazePinpointLocalizer p = BlazeSession.pinpoint();
            return p == null ? Double.NaN : p.usingSdk ? 1 : 0;
        });
    }

    public static void drivetrain(Recorder r, final Drivetrain d) {

        if (d == null) return;
        r.flag("drive", "holding", "pose hold on and trusting its pose", d::isHolding);
        r.channel("drive", "cmd_forward", "", "what it asked for (-1..1)", () -> d.getCommand()[0]);
        r.channel("drive", "cmd_strafe", "", "", () -> d.getCommand()[1]);
        r.channel("drive", "cmd_turn", "", "", () -> d.getCommand()[2]);
        r.channel("drive", "wheel_lf", "", "power sent", () -> d.getWheels()[0]);
        r.channel("drive", "wheel_rf", "", "", () -> d.getWheels()[1]);
        r.channel("drive", "wheel_lb", "", "", () -> d.getWheels()[2]);
        r.channel("drive", "wheel_rb", "", "", () -> d.getWheels()[3]);
        final DriveController c = d.getController();
        if (c == null) return;
        r.channel("drive", "ref_x", "in", "pose hold reference", c::referenceX);
        r.channel("drive", "ref_y", "in", "", c::referenceY);
        r.channel("drive", "ref_heading", "deg", "", () -> Math.toDegrees(c.referenceHeading()));
        r.channel("drive", "err_forward", "", "hold error, forward axis", () -> c.error[0]);
        r.channel("drive", "err_strafe", "", "", () -> c.error[1]);
        r.channel("drive", "err_turn", "", "", () -> c.error[2]);
        r.channel("drive", "power_forward", "", "hold output", () -> c.power[0]);
        r.channel("drive", "power_strafe", "", "", () -> c.power[1]);
        r.channel("drive", "power_turn", "", "", () -> c.power[2]);
    }

    public static void turret(Recorder r, final TurretBase t) {

        if (t == null) return;
        final TurretController c = t.getController();
        r.channel("turret", "target", "deg", "where it's told to go", () -> (t.getTargetPosition() - t.startPosition) / tpd());
        r.channel("turret", "position", "deg", "encoder", () -> (t.getCurrentPosition() - t.startPosition) / tpd());
        r.channel("turret", "error", "deg", "target - position", () -> (t.getTargetPosition() - t.getCurrentPosition()) / tpd());
        r.channel("turret", "aim_error", "deg", "what the aim gate checks (smoothed)", t::getAimErrorDegrees);
        r.channel("turret", "velocity", "deg/s", "filtered", () -> t.getVelocity() / tpd());
        r.channel("turret", "raw_velocity", "deg/s", "encoder", () -> t.getRawVelocity() / tpd());
        r.channel("turret", "target_velocity", "deg/s", "aim rate feedforward (turns against the robot's spin)", () -> t.getTargetVelocity() / tpd());
        r.channel("turret", "pid_velocity", "deg/s", "position PID output", () -> c.positionalVelocity / tpd());
        r.channel("turret", "power", "", "servo power (-1..1)", t::getPower);
        r.channel("turret", "volts", "V", "sent, after the limits", t::getVolts);
        r.channel("turret", "servo_left", "", "", () -> t.getServoPowers()[0]);
        r.channel("turret", "servo_right", "", "", () -> t.getServoPowers()[1]);
        r.channel("turret", "supply_v", "V", "what the volts are scaled against", t::getCompensationVolts);
        r.channel("turret", "p", "V", "", () -> t.p);
        r.channel("turret", "i", "V", "", () -> t.i);
        r.channel("turret", "d", "V", "", () -> t.d);
        r.channel("turret", "f", "V", "friction push + hold", () -> t.f);
        r.channel("turret", "v", "V", "aim rate feedforward", () -> t.v);
        r.channel("turret", "friction", "V", "friction push alone", () -> c.s);
        r.channel("turret", "hold", "V", "hold push alone", () -> c.f);
        r.flag("turret", "on_target", "aim gate", t::isOnTarget);
        r.flag("turret", "encoder_frozen", "cut off for good this OpMode", t::isEncoderFrozen);
        r.channel("turret", "pulsing", "0/1", "holding with timed pushes (small error, still aim)", () -> c.pulsing ? 1 : 0);
        r.channel("turret", "pulses_no_count", "", "pushes in a row without an encoder count", () -> c.pulsesWithoutCounts);
        r.channel("turret", "friction_trim", "V", "friction learned this match on top of the tuned breakaway", () -> c.frictionTrim);
        r.channel("turret", "pulse_length", "s", "learned push length", () -> c.pulseLength);
        r.channel("turret", "loop_dt", "s", "its own update gap", t::getLoopDt);
        r.channel("turret", "filter_hz", "Hz", "velocity filter", t::getFilterBandwidth);
    }

    public static void flywheel(Recorder r, final Flywheel f) {

        if (f == null) return;
        r.channel("flywheel", "target", "ticks/s", "", f::getTargetVelocity);
        r.channel("flywheel", "velocity", "ticks/s", "filtered", f::getCurrentVelocity);
        r.channel("flywheel", "raw_velocity", "ticks/s", "", f::getRawVelocity);
        r.channel("flywheel", "estimate", "ticks/s", "", f::getVelocityEstimate);
        r.channel("flywheel", "error", "ticks/s", "", f::getError);
        r.channel("flywheel", "power", "", "sent", f::getPower);
        r.channel("flywheel", "volts", "V", "filtered supply", f::getFilteredVolts);
        r.channel("flywheel", "p", "", "", () -> f.p);
        r.channel("flywheel", "v", "", "", () -> f.v);
        r.channel("flywheel", "s", "", "", () -> f.s);
        r.channel("flywheel", "load", "", "", () -> f.load);
        r.channel("flywheel", "boost", "", "", () -> f.boost);
        r.channel("flywheel", "feed_boost_v", "V", "", () -> f.feedBoostVolts);
        r.flag("flywheel", "feeding", "", f::isFeeding);
        r.channel("flywheel", "shots_seen", "", "count", () -> f.shotsSeen);
        r.channel("flywheel", "loop_dt", "s", "", f::getLoopDt);
        r.channel("flywheel", "filter_hz", "Hz", "", () -> f.getEncoder().getFilterBandwidth());
    }

    public static void shooter(Recorder r, final Shooter s) {

        if (s == null) return;
        r.flag("shooter", "on", "flywheel toggle", s::isShooterOn);
        r.flag("shooter", "auto_aim", "", s::isAutoAimOn);
        r.flag("shooter", "locked", "turret held at 0 (gamepad 1 Y)", s::isTurretLocked);
        r.flag("shooter", "aim_usable", "inside the travel, not mid-swing", s::isAimSolutionUsable);
        r.flag("shooter", "close_cell", "aiming at the CLOSE cell", s::isCloseCell);
        r.channel("shooter", "aim_raw", "deg", "aim before routing (-180..180)", () -> s.rawAimDegrees);
        r.channel("shooter", "aim", "deg", "aim after routing (the turret's target)", () -> s.tt);
        r.channel("shooter", "aim_rate", "deg/s", "", () -> s.aimRateDegrees);
        r.channel("shooter", "goal_x", "in", "", () -> s.goalCoordinate != null ? s.goalCoordinate.x() : Double.NaN);
        r.channel("shooter", "goal_y", "in", "", () -> s.goalCoordinate != null ? s.goalCoordinate.y() : Double.NaN);
        r.channel("shooter", "virtual_goal_x", "in", "goal - v T (SOTM)", () -> s.virtualGoal != null ? s.virtualGoal.x() : Double.NaN);
        r.channel("shooter", "virtual_goal_y", "in", "", () -> s.virtualGoal != null ? s.virtualGoal.y() : Double.NaN);
        r.channel("shooter", "turret_x", "in", "turret on the field", () -> s.turretPose != null ? s.turretPose.x() : Double.NaN);
        r.channel("shooter", "turret_y", "in", "", () -> s.turretPose != null ? s.turretPose.y() : Double.NaN);
        r.channel("shooter", "distance", "in", "turret to the (virtual) goal", () -> s.distanceToGoal);
        r.channel("shooter", "flywheel_target", "ticks/s", "what the aim asked for", () -> s.flywheelTargetVelocity);
        r.channel("shooter", "manual_velocity", "ticks/s", "tuner/auto override (NaN = none)", () -> s.manualFlywheelVelocity);
        r.channel("shooter", "shot_distance", "in", "still distance the flywheel speed is for", () -> s.shotDistance);
        r.channel("shooter", "shot_time", "s", "flight time used", () -> s.shotTime);
        r.channel("shooter", "shot_miss", "in", "+ long / - short with the speed it's at", () -> s.shotMissInches);
        r.flag("shooter", "will_score", "miss gate", Shooter::shotWillScore);
        r.channel("shooter", "swings", "", "count: full-turn swings", s::getSwings);
        r.channel("shooter", "relocalizations", "", "count", s::getRelocalizations);
    }

    public static void blocker(Recorder r, final Blocker b, boolean gates) {

        if (b == null) return;
        r.flag("blocker", "clear", "open: balls go to the flywheel", () -> b.getState() == Blocker.BlockerState.CLEAR);
        r.channel("blocker", "position", "", "servo (its state's)", () -> b.getState().getPosition());
        if (!gates) return;
        r.flag("blocker", "flywheel_ready", "shot gate", b::isFlywheelReady);
        r.flag("blocker", "aim_ready", "aim gate", b::isAimReady);
    }

    public static void intake(Recorder r, final Intake i) {

        if (i == null) return;
        r.channel("intake", "state", "", "1 in, -1 reverse, 0 idle", () -> i.getState() == Intake.IntakeState.INTAKE ? 1 : i.getState() == Intake.IntakeState.REVERSE_INTAKE ? -1 : 0);
        r.channel("intake", "target_velocity", "ticks/s", "its state's", () -> i.getState().getVelocity());
    }

    public static void veloMotor(Recorder r, String group, final GeneralVeloMotor m) {

        if (m == null) return;
        r.channel(group, "velocity", "ticks/s", "", m::getVelocity);
        r.channel(group, "power", "", "", m::getPower);
        r.channel(group, "p", "", "", () -> m.p);
        r.channel(group, "d", "", "", () -> m.d);
        r.channel(group, "ff", "", "", () -> m.ff);
    }

    public static void fiArm(Recorder r, final FIArm a) {

        if (a == null) return;
        r.flag("fiarm", "deployed", "", () -> a.getState() == FIArm.FIArmState.DEPLOY);
        r.channel("fiarm", "left", "", "servo (its state's)", () -> a.getState().getLeftPosition());
        r.channel("fiarm", "right", "", "servo (its state's)", () -> a.getState().getRightPosition());
    }

    public static void context(Recorder r, String opMode, String alliance, HardwareMap hardwareMap) {

        r.meta("opmode", opMode);
        r.meta("alliance", alliance);
        r.meta("started", new Date());
        r.meta("start_from", AutoHandoff.lastResult);
        r.meta("turret_ticks_per_degree", ShooterConstants.TURRET_TICKS_PER_DEGREE);
        for (String name : CONSTANTS) constants(r, "org.firstinspires.ftc.teamcode.Constants." + name);
        if (hardwareMap == null) return;
        try {
            for (HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
                StringBuilder names = new StringBuilder();
                for (String n : hardwareMap.getNamesOf(device)) names.append(names.length() > 0 ? "/" : "").append(n);
                r.meta("hw." + names, device.getDeviceName() + " | " + device.getConnectionInfo() + " | v" + device.getVersion());
            }
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) r.meta("hub." + hub.getModuleAddress(), (hub.isParent() ? "parent " : "child ") + hub.getBulkCachingMode());
        } catch (Exception e) {
            r.meta("hw.error", e.toString());
        }
    }

    private static void constants(Recorder r, String className) {

        try {
            Class<?> c = Class.forName(className);
            String prefix = className.substring(className.lastIndexOf("Constants.") + "Constants.".length()) + ".";
            for (Field field : c.getDeclaredFields()) {
                if (!Modifier.isStatic(field.getModifiers())) continue;
                field.setAccessible(true);
                Object value = field.get(null);
                String text;
                if (value instanceof double[]) text = Arrays.toString((double[]) value);
                else if (value instanceof int[]) text = Arrays.toString((int[]) value);
                else if (value instanceof Object[]) text = Arrays.deepToString((Object[]) value);
                else text = String.valueOf(value);
                r.meta(prefix + field.getName(), text);
            }
        } catch (Throwable e) {
            r.meta("constants." + className, "unreadable: " + e);
        }
    }
}
