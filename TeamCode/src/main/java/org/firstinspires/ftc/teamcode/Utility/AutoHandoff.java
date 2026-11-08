package org.firstinspires.ftc.teamcode.Utility;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;

/// what the last auto left for TeleOp: where the robot is and how far the turret is from home. TeleOp's turret otherwise
/// takes wherever it is at INIT as home. Statics, so it lives as long as the robot app (a restart or Sloth reload clears it).
/// The autos write it every loop once started, and once more when they stop. TeleOpBaseOpMode takes it once at INIT
public final class AutoHandoff {

    //true = an auto wrote this and no TeleOp has used it yet. TeleOp sets it false whether it used it or not
    public static boolean infoFromAuto;
    //robot pose (in, in, rad) and turret ticks from home, when they were saved (Clock s) and by which auto
    public static double x, y, heading, turretTicks, savedAt = Double.NaN;
    public static String auto = "";
    //older than this it's ignored: someone could have turned the turret by hand since
    public static double MAX_AGE_SECONDS = 180;

    public static String lastResult = "nothing from an auto";

    private AutoHandoff() {}

    /// autos: no allocation, called every loop
    public static void record(String autoName, Pose pose, double turretTicksFromHome) {

        if (pose == null || !MathUtil.isFinite(pose.x()) || !MathUtil.isFinite(pose.y()) || !MathUtil.isFinite(pose.heading()) || !MathUtil.isFinite(turretTicksFromHome)) return;
        x = pose.x();
        y = pose.y();
        heading = pose.heading();
        turretTicks = turretTicksFromHome;
        savedAt = Clock.seconds();
        auto = autoName;
        infoFromAuto = true;
    }

    /// TeleOp INIT: true if there's a fresh, sane handoff to use (then read x, y, heading, turretTicks). Clears the flag either
    /// way so the next OpMode can't use it again
    public static boolean take() {

        boolean fresh = infoFromAuto;
        infoFromAuto = false;
        double age = Clock.seconds() - savedAt;
        double degrees = ShooterConstants.TURRET_AIM_DIRECTION * turretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE;
        double margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
        if (!fresh) lastResult = "nothing from an auto: turret home = where it is now";
        else if (!(age >= 0 && age <= MAX_AGE_SECONDS)) lastResult = String.format(java.util.Locale.US, "%s was %.0f s ago, ignored: turret home = where it is now", auto, age);
        else if (degrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin || degrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin) lastResult = String.format(java.util.Locale.US, "%s turret %.0f deg is past the travel, ignored", auto, degrees);
        else {
            lastResult = String.format(java.util.Locale.US, "from %s (%.0f s ago): x %.1f y %.1f heading %.0f deg, turret %.1f deg from home", auto, age, x, y, Math.toDegrees(heading), degrees);
            return true;
        }
        return false;
    }

    public static Pose pose() {
        return new Pose(x, y, heading);
    }
}
