package org.firstinspires.ftc.teamcode.Utility;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;

public final class AutoHandoff {

    public static boolean infoFromAuto;
    public static double x, y, heading, turretTicks, savedAt = Double.NaN;
    public static String auto = "";
    public static double MAX_AGE_SECONDS = 180;

    public static String lastResult = "nothing from an auto";

    private AutoHandoff() {}

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
