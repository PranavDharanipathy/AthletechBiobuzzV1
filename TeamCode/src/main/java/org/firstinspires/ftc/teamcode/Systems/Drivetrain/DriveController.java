package org.firstinspires.ftc.teamcode.Systems.Drivetrain;

public class DriveController {

    public static final int FORWARD = 0, LEFT = 1, TURN = 2;

    public final double[] kS = {0.06, 0.08, 0.07}, kV = {0.0107, 0.0131, 0.105}, kA = {0.0035, 0.0045, 0.03};
    public final double[] maxAccel = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY}, maxDecel = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    public final double[] bandwidth = {6, 6, 6};
    public double damping = 1, integralRatio = 0.25;
    public final double[] maxIntegral = {0.3, 0.3, 0.3}, leash = {4, 4, Math.toRadians(20)};
    public double accelFraction = 0.9;
    public double maxCorrection = 1, nominalVolts = 12;
    public double jumpInches = 6, jumpRadians = Math.toRadians(20);

    public final double[] kP = new double[3], kD = new double[3], kI = new double[3];
    public final double[] error = new double[3], velocityError = new double[3], feedforward = new double[3], feedback = new double[3], power = new double[3];

    private double refX, refY, refH;
    private final double[] refV = new double[3], lastRefV = new double[3], integral = new double[3], refRobot = new double[3];
    private final boolean[] slowing = new boolean[3];
    private double lastX, lastY, lastH;
    private boolean started;

    public void reset() {
        started = false;
    }

    public double maxVelocity(int axis) {
        return Math.max(0, 1 - kS[axis]) / kV[axis];
    }

    public void computeGains() {

        for (int a = 0; a < 3; a++) {
            double w = bandwidth[a], tau = kA[a] / kV[a], z = damping, r = integralRatio;
            kP[a] = (1 + 2 * z * r) * w * w * tau * kV[a];
            kD[a] = Math.max(0, ((2 * z + r) * w * tau - 1) * kV[a]);
            kI[a] = r * w * w * w * tau * kV[a];
        }
    }

    public boolean update(double dt, double[] command, double x, double y, double heading, double vx, double vy, double omega, double volts) {

        if (!(finite(x) && finite(y) && finite(heading) && finite(vx) && finite(vy) && finite(omega))) {
            for (int a = 0; a < 3; a++) power[a] = command[a];
            started = false;
            return false;
        }
        if (!(dt > 0) || dt > 0.25) dt = 0.02;
        if (!started) {
            refX = lastX = x;
            refY = lastY = y;
            refH = lastH = heading;
            for (int a = 0; a < 3; a++) refV[a] = lastRefV[a] = integral[a] = 0;
            started = true;
        }
        double jumpX = x - lastX - vx * dt, jumpY = y - lastY - vy * dt, jumpH = wrap(heading - lastH - omega * dt);
        if (Math.hypot(jumpX, jumpY) > jumpInches || Math.abs(jumpH) > jumpRadians) {
            refX += x - lastX;
            refY += y - lastY;
            refH += wrap(heading - lastH);
        }
        lastX = x;
        lastY = y;
        lastH = heading;
        computeGains();

        for (int a = 0; a < 3; a++) {
            double want = clamp(command[a], -1, 1) * maxVelocity(a);
            double v = refV[a];
            boolean speedingUp = Math.abs(want) > Math.abs(v) && want * v >= 0;
            double headroom = speedingUp ? 1 - kS[a] - kV[a] * Math.abs(v) : 1 - kS[a] + kV[a] * Math.abs(v);
            double step = accelFraction * Math.min(Math.max(headroom, 0.05) / kA[a], speedingUp ? maxAccel[a] : maxDecel[a]) * dt;
            lastRefV[a] = v;
            refV[a] = v + clamp(want - v, -step, step);
            slowing[a] = want * v < 0 || Math.abs(want) < Math.abs(v) - 1e-9;
        }
        refH += refV[TURN] * dt;
        double c = Math.cos(refH), s = Math.sin(refH);
        refX += (c * refV[FORWARD] - s * refV[LEFT]) * dt;
        refY += (s * refV[FORWARD] + c * refV[LEFT]) * dt;

        double ex = refX - x, ey = refY - y, distance = Math.hypot(ex, ey);
        double maxDistance = Math.min(leash[FORWARD], leash[LEFT]);
        if (distance > maxDistance) {
            refX = x + ex * maxDistance / distance;
            refY = y + ey * maxDistance / distance;
            ex = refX - x;
            ey = refY - y;
        }
        double eh = wrap(refH - heading);
        if (Math.abs(eh) > leash[TURN]) {
            refH = heading + Math.signum(eh) * leash[TURN];
            eh = refH - heading;
        }

        double ch = Math.cos(heading), sh = Math.sin(heading);
        error[FORWARD] = ch * ex + sh * ey;
        error[LEFT] = -sh * ex + ch * ey;
        error[TURN] = eh;
        if (slowing[FORWARD]) {
            refX -= ch * error[FORWARD];
            refY -= sh * error[FORWARD];
            error[FORWARD] = 0;
        }
        if (slowing[LEFT]) {
            refX += sh * error[LEFT];
            refY -= ch * error[LEFT];
            error[LEFT] = 0;
        }
        if (slowing[TURN]) {
            refH = heading;
            error[TURN] = 0;
        }
        double dh = refH - heading, cd = Math.cos(dh), sd = Math.sin(dh);
        double refForward = cd * refV[FORWARD] - sd * refV[LEFT], refLeft = sd * refV[FORWARD] + cd * refV[LEFT];
        velocityError[FORWARD] = refForward - (ch * vx + sh * vy);
        velocityError[LEFT] = refLeft - (-sh * vx + ch * vy);
        velocityError[TURN] = refV[TURN] - omega;
        refRobot[FORWARD] = refForward;
        refRobot[LEFT] = refLeft;
        refRobot[TURN] = refV[TURN];

        double scale = volts > 1 ? nominalVolts / volts : 1;
        for (int a = 0; a < 3; a++) {
            double accel = (refV[a] - lastRefV[a]) / dt;
            feedforward[a] = kS[a] * smoothSign(refRobot[a], 0.05 * maxVelocity(a)) + kV[a] * refRobot[a] + kA[a] * accel;
            double pd = kP[a] * error[a] + kD[a] * velocityError[a];
            double unclamped = feedforward[a] + pd + kI[a] * integral[a];
            boolean saturated = Math.abs(unclamped) * scale >= 1 && Math.signum(unclamped) == Math.signum(error[a]);
            if (!saturated) integral[a] += error[a] * dt;
            if (kI[a] > 0) integral[a] = clamp(integral[a], -maxIntegral[a] / kI[a], maxIntegral[a] / kI[a]);
            feedback[a] = clamp(pd + kI[a] * integral[a], -maxCorrection, maxCorrection);
            power[a] = clamp((feedforward[a] + feedback[a]) * scale, -1, 1);
        }
        return true;
    }

    public double referenceX() {
        return refX;
    }

    public double referenceY() {
        return refY;
    }

    public double referenceHeading() {
        return refH;
    }

    public static void wheels(double forward, double left, double ccw, double[] out) {

        double lf = forward - left - ccw, rf = forward + left + ccw, lb = forward + left - ccw, rb = forward - left + ccw;
        double m = Math.max(1, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));
        out[0] = lf / m;
        out[1] = rf / m;
        out[2] = lb / m;
        out[3] = rb / m;
    }

    static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    static double smoothSign(double x, double width) {
        return width > 0 ? Math.tanh(x / width) : Math.signum(x);
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    static boolean finite(double x) {
        return !Double.isNaN(x) && !Double.isInfinite(x);
    }
}
