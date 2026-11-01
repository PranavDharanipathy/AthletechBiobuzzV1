package org.firstinspires.ftc.teamcode.Testing.ShotExperiment;

/** Vacuum reference only. Does not choose servo positions or command RPM; no calibrated aero model. */
public final class PollenPhysics {
    private PollenPhysics() { }
    public static double releaseHeight(double degrees) {
        return .0254*(9.30+2.82*Math.cos(Math.toRadians(degrees)));
    }
    public static double vacuumSpeed(double distanceMeters, double angleDegrees) {
        if (!Double.isFinite(distanceMeters) || distanceMeters<=0 || !Double.isFinite(angleDegrees)
                || angleDegrees<31 || angleDegrees>54) return Double.NaN;
        double theta=Math.toRadians(angleDegrees);
        double denominator=2*Math.pow(Math.cos(theta),2)
                *(distanceMeters*Math.tan(theta)-(59.55*.0254-releaseHeight(angleDegrees)));
        return denominator>0 ? Math.sqrt(9.80665*distanceMeters*distanceMeters/denominator) : Double.NaN;
    }
    public static double wheelRpm(double speed, double speedToSurfaceRatio) {
        if (!Double.isFinite(speed) || speed<=0 || !Double.isFinite(speedToSurfaceRatio)
                || speedToSurfaceRatio<=0 || speedToSurfaceRatio>.5) return Double.NaN;
        return 60*speed/(speedToSurfaceRatio*Math.PI*.072);
    }
}
