package org.athletech.sim.undeterminables;

import org.athletech.sim.utility.Rng;

/**
 * How a dead-reckoning localizer (odometry pods / drive encoders + IMU) drifts from the truth.
 *
 * <ul>
 *   <li>Scale error: each run gets a fixed calibration error per axis (e.g. pod wheel diameter off
 *       by 0.5 %), so error grows linearly with distance.</li>
 *   <li>Slip noise: random error that grows with the square root of distance (bumps, scrub).</li>
 *   <li>Gyro: fixed bias per run (deg/s), white noise (deg/sqrt(s)) and a scale error on turns.
 *       Heading error rotates every later move, so it turns into position error.</li>
 *   <li>Vision fixes (optional): every {@code visionIntervalS} the estimate is reset to the truth
 *       plus {@code visionStdM} of noise (AprilTag relocalization).</li>
 * </ul>
 */
public final class OdometryErrorModel {

    public static final class Params {
        private final double scaleStd;
        private final double slipStdPerSqrtM;
        private final double gyroBiasStdDps;
        private final double gyroNoiseDegPerSqrtS;
        private final double gyroScaleStd;
        private final double visionStdM;
        private final double visionIntervalS;

        public Params(double scaleStd, double slipStdPerSqrtM, double gyroBiasStdDps, double gyroNoiseDegPerSqrtS, double gyroScaleStd, double visionStdM, double visionIntervalS) {
            this.scaleStd = scaleStd;
            this.slipStdPerSqrtM = slipStdPerSqrtM;
            this.gyroBiasStdDps = gyroBiasStdDps;
            this.gyroNoiseDegPerSqrtS = gyroNoiseDegPerSqrtS;
            this.gyroScaleStd = gyroScaleStd;
            this.visionStdM = visionStdM;
            this.visionIntervalS = visionIntervalS;
        }

        public double scaleStd() { return scaleStd; }
        public double slipStdPerSqrtM() { return slipStdPerSqrtM; }
        public double gyroBiasStdDps() { return gyroBiasStdDps; }
        public double gyroNoiseDegPerSqrtS() { return gyroNoiseDegPerSqrtS; }
        public double gyroScaleStd() { return gyroScaleStd; }
        public double visionStdM() { return visionStdM; }
        public double visionIntervalS() { return visionIntervalS; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Params)) return false;
            Params o = (Params) other;
            return Double.compare(scaleStd, o.scaleStd) == 0
                    && Double.compare(slipStdPerSqrtM, o.slipStdPerSqrtM) == 0
                    && Double.compare(gyroBiasStdDps, o.gyroBiasStdDps) == 0
                    && Double.compare(gyroNoiseDegPerSqrtS, o.gyroNoiseDegPerSqrtS) == 0
                    && Double.compare(gyroScaleStd, o.gyroScaleStd) == 0
                    && Double.compare(visionStdM, o.visionStdM) == 0
                    && Double.compare(visionIntervalS, o.visionIntervalS) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(scaleStd, slipStdPerSqrtM, gyroBiasStdDps, gyroNoiseDegPerSqrtS, gyroScaleStd, visionStdM, visionIntervalS);
        }

        @Override
        public String toString() {
            return "Params[scaleStd=" + scaleStd + ", slipStdPerSqrtM=" + slipStdPerSqrtM + ", gyroBiasStdDps=" + gyroBiasStdDps + ", gyroNoiseDegPerSqrtS=" + gyroNoiseDegPerSqrtS + ", gyroScaleStd=" + gyroScaleStd + ", visionStdM=" + visionStdM + ", visionIntervalS=" + visionIntervalS + "]";
        }
    }

    private final Params p;
    private final Rng rng;
    private final double scaleF, scaleL, biasRadPerS, gyroScale;
    private double estX, estY, estH;
    private double sinceFix;

    public OdometryErrorModel(Params p, Rng rng, double x, double y, double heading) {
        this.p = p;
        this.rng = rng;
        this.scaleF = rng.gaussian(0, nz(p.scaleStd()));
        this.scaleL = rng.gaussian(0, nz(p.scaleStd()));
        this.biasRadPerS = Math.toRadians(rng.gaussian(0, nz(p.gyroBiasStdDps())));
        this.gyroScale = rng.gaussian(0, nz(p.gyroScaleStd()));
        estX = x;
        estY = y;
        estH = heading;
    }

    private static double nz(double v) {
        return Double.isNaN(v) ? 0 : v;
    }

    /**
     * Feeds one true motion increment in the robot frame (forward, left, yaw change) over
     * {@code dt}. {@code trueX/Y/H} is the true pose after the move (used for vision fixes).
     */
    public void step(double dForward, double dLeft, double dYaw, double dt, double trueX, double trueY, double trueH) {
        double yaw = dYaw * (1 + gyroScale) + biasRadPerS * dt
                + Math.toRadians(nz(p.gyroNoiseDegPerSqrtS())) * Math.sqrt(dt) * rng.gaussian();
        double f = dForward * (1 + scaleF) + nz(p.slipStdPerSqrtM()) * Math.sqrt(Math.abs(dForward)) * rng.gaussian();
        double l = dLeft * (1 + scaleL) + nz(p.slipStdPerSqrtM()) * Math.sqrt(Math.abs(dLeft)) * rng.gaussian();
        double mid = estH + yaw / 2;
        estX += f * Math.cos(mid) - l * Math.sin(mid);
        estY += f * Math.sin(mid) + l * Math.cos(mid);
        estH += yaw;
        sinceFix += dt;
        if (p.visionIntervalS() > 0 && !Double.isNaN(p.visionStdM()) && sinceFix >= p.visionIntervalS()) {
            estX = trueX + rng.gaussian(0, p.visionStdM());
            estY = trueY + rng.gaussian(0, p.visionStdM());
            estH = trueH; // AprilTag heading is far better than the gyro drift being modelled
            sinceFix = 0;
        }
    }

    public double estX() { return estX; }
    public double estY() { return estY; }
    public double estHeading() { return estH; }
}
