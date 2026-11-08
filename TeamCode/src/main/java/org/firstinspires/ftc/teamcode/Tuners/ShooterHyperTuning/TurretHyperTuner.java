package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import org.firstinspires.ftc.teamcode.Constants.TurretConstants;

public class TurretHyperTuner {

    public final OnlineTuner moves;
    public double rateScale = TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE;
    public double jumpDegrees = 15, stillRobot = 30, reachBand = 1, shakeTime = 0.4, moveTimeout = 2.5;
    public double spinRate = 60, trackWindow = 1, rateLearnRate = 0.5, rateLag = 0.1;
    public double lastMoveCost = Double.NaN, lastTrackBias = Double.NaN, lastTrackRms = Double.NaN;
    public int movesScored;

    private boolean moving;
    private double moveFrom, moveTo, moveStart, reachedAt, overshoot, flips, shakeSum, shakeN, lastVolts, lastTarget = Double.NaN;
    private double trackStart = Double.NaN, biasSum, absRateSum, errSq, trackN;

    public TurretHyperTuner() {
        moves = new OnlineTuner("turret moves", 3, new OnlineTuner.Knob("bandwidth", TurretConstants.TURRET_BANDWIDTH, 1.5, 10, 0.6, 0.1), new OnlineTuner.Knob("brake_degrees", TurretConstants.TURRET_BRAKE_DEGREES, 0, 12, 1.5, 0.25), new OnlineTuner.Knob("max_speed_fraction", TurretConstants.TURRET_MAX_SPEED_FRACTION, 0.6, 0.95, 0.05, 0.01), new OnlineTuner.Knob("acceleration_fraction", TurretConstants.TURRET_ACCELERATION_FRACTION, 0.5, 0.95, 0.05, 0.01), new OnlineTuner.Knob("resync_degrees", TurretConstants.TURRET_RESYNC_DEGREES, 3, 20, 2, 0.5), new OnlineTuner.Knob("landing", TurretConstants.TURRET_LANDING_ENABLED ? 1 : 0, 0, 1, 1, 1));
    }

    public void apply() {

        TurretConstants.TURRET_BANDWIDTH = moves.value("bandwidth");
        TurretConstants.TURRET_BRAKE_DEGREES = moves.value("brake_degrees");
        TurretConstants.TURRET_MAX_SPEED_FRACTION = moves.value("max_speed_fraction");
        TurretConstants.TURRET_ACCELERATION_FRACTION = moves.value("acceleration_fraction");
        TurretConstants.TURRET_RESYNC_DEGREES = moves.value("resync_degrees");
        TurretConstants.TURRET_LANDING_ENABLED = moves.value("landing") >= 0.5;
        TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE = rateScale;
    }

    public boolean update(double now, double target, double position, double velocity, double volts, double targetRate, double robotOmega, boolean aimUsable) {

        boolean finished = false;
        boolean jumped = !Double.isNaN(lastTarget) && Math.abs(target - lastTarget) > jumpDegrees;
        lastTarget = target;
        if (jumped) {
            moving = Math.abs(robotOmega) < stillRobot;
            moveFrom = position;
            moveTo = target;
            moveStart = now;
            reachedAt = Double.NaN;
            overshoot = flips = shakeSum = shakeN = 0;
        }
        else if (moving && Math.abs(target - moveTo) > reachBand) moving = false;
        if (moving) {
            double direction = Math.signum(moveTo - moveFrom), error = moveTo - position;
            overshoot = Math.max(overshoot, -error * direction);
            if (Math.abs(velocity) > 60 && Math.abs(error) > 20 && volts * lastVolts < 0) flips++;
            if (Double.isNaN(reachedAt) && Math.abs(error) < reachBand) reachedAt = now;
            if (!Double.isNaN(reachedAt)) {
                shakeSum += velocity * velocity;
                shakeN++;
            }
            boolean done = !Double.isNaN(reachedAt) && now - reachedAt >= shakeTime;
            if (done || now - moveStart > moveTimeout) {
                double reach = Double.isNaN(reachedAt) ? moveTimeout : reachedAt - moveStart;
                double shake = shakeN > 0 ? Math.sqrt(shakeSum / shakeN) : 0;
                lastMoveCost = reach + 0.03 * overshoot + 0.1 * flips + 0.01 * shake;
                movesScored++;
                finished = moves.submit(lastMoveCost);
                moving = false;
            }
        }
        lastVolts = volts;

        if (aimUsable && Math.abs(targetRate) > spinRate && !jumped) {
            if (Double.isNaN(trackStart)) trackStart = now;
            double error = target - position;
            biasSum += error * Math.signum(targetRate);
            absRateSum += Math.abs(targetRate);
            errSq += error * error;
            trackN++;
            if (now - trackStart >= trackWindow && trackN > 10) {
                lastTrackBias = biasSum / trackN;
                lastTrackRms = Math.sqrt(errSq / trackN);
                double meanRate = absRateSum / trackN;
                rateScale = clamp(rateScale + rateLearnRate * lastTrackBias / (meanRate * rateLag), 0.7, 1.3);
                trackStart = Double.NaN;
                biasSum = absRateSum = errSq = trackN = 0;
            }
        }
        else {
            trackStart = Double.NaN;
            biasSum = absRateSum = errSq = trackN = 0;
        }
        return finished;
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
