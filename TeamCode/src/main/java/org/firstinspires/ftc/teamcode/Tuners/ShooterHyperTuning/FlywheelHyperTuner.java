package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelHyperTuner {

    public final OnlineTuner stability, shots;
    public double steadyBand = 40, steadyWindow = 1.5, quietAfterFeed = 0.6, recoveredBand = 30, recoveredTime = 0.15, dropWatch = 0.25;
    public double recoveryWeight = 300, dipWeight = 0.5, targetStep = 10;
    public double lastSteadyCost = Double.NaN, lastShotCost = Double.NaN;
    public int bursts;

    private double steadyStart = Double.NaN, sumE2, sumJ2, steadyN, lastVelocity = Double.NaN, lastFeedEnd = -1e9, lastTarget;
    private boolean inBurst, wasFeeding, watching;
    private double feedEnd = Double.NaN, recoveredSince = Double.NaN, watchUntil, minSince;
    private double contactSum, contactSq, contacts, dipSum;
    private int lastShots;

    public FlywheelHyperTuner(FlywheelPIDVSCoefficients start, double feedBoostVolts, double shotKickVolts, double shotKickTime, double preKickVolts, double preKickLead) {

        stability = new OnlineTuner("flywheel stability", 2, new OnlineTuner.Knob("kp_close", start.kpClose, 0.2 * start.kpClose, 3 * start.kpClose, 0.15 * start.kpClose, 0.02 * start.kpClose), new OnlineTuner.Knob("kd", start.kd, 0, Math.max(5 * start.kd, 0.002), Math.max(0.3 * start.kd, 0.0002), 0.00002));
        shots = new OnlineTuner("flywheel shots", 2, new OnlineTuner.Knob("feed_boost_volts", feedBoostVolts, 0, 3, 0.4, 0.05), new OnlineTuner.Knob("shot_kick_volts", shotKickVolts, 0, 6, 0.8, 0.1), new OnlineTuner.Knob("shot_kick_time", shotKickTime, 0.02, 0.3, 0.04, 0.01), new OnlineTuner.Knob("kp_far", start.kpFar, 0.3 * start.kpFar, 3 * start.kpFar, 0.2 * start.kpFar, 0.02 * start.kpFar), new OnlineTuner.Knob("max_p", start.maxP, 2, 12, 1, 0.25), new OnlineTuner.Knob("pre_kick_volts", preKickVolts, 0, 4, 0.5, 0.1), new OnlineTuner.Knob("pre_kick_lead", preKickLead, 0, 0.15, 0.03, 0.01));
    }

    public void apply(Flywheel flywheel, FlywheelPIDVSCoefficients coefficients) {

        coefficients.kpClose = stability.value("kp_close");
        coefficients.kpFar = shots.value("kp_far");
        flywheel.kd = stability.value("kd");
        flywheel.maxP = shots.value("max_p");
        flywheel.feedBoostVolts = shots.value("feed_boost_volts");
        flywheel.shotKickVolts = shots.value("shot_kick_volts");
        flywheel.shotKickTime = shots.value("shot_kick_time");
        flywheel.preKickVolts = shots.value("pre_kick_volts");
        flywheel.preKickLead = shots.value("pre_kick_lead");
    }

    public boolean update(double now, double target, double velocity, boolean feeding, int shotsSeen) {

        boolean finished = false;
        double error = target - velocity;
        if (Math.abs(target - lastTarget) > targetStep) resetSteady();
        lastTarget = target;
        if (feeding) lastFeedEnd = now;

        if (target > 0 && !feeding && now - lastFeedEnd > quietAfterFeed && Math.abs(error) < steadyBand && !Double.isNaN(lastVelocity)) {
            if (Double.isNaN(steadyStart)) steadyStart = now;
            double jitter = velocity - lastVelocity;
            sumE2 += error * error;
            sumJ2 += jitter * jitter;
            steadyN++;
            if (now - steadyStart >= steadyWindow && steadyN > 10) {
                lastSteadyCost = Math.sqrt(sumE2 / steadyN) + 0.5 * Math.sqrt(sumJ2 / steadyN);
                finished |= stability.submit(lastSteadyCost);
                resetSteady();
            }
        }
        else resetSteady();

        if (target > 0 && feeding && !inBurst) {
            inBurst = true;
            contactSum = contactSq = contacts = dipSum = 0;
            lastShots = shotsSeen;
            watching = false;
        }
        if (inBurst) {
            if (shotsSeen > lastShots && !Double.isNaN(lastVelocity)) {
                double atContact = lastVelocity - target;
                contactSum += atContact;
                contactSq += atContact * atContact;
                contacts++;
                watching = true;
                watchUntil = now + dropWatch;
                minSince = velocity;
                lastShots = shotsSeen;
            }
            if (watching) {
                minSince = Math.min(minSince, velocity);
                if (now >= watchUntil) {
                    dipSum += Math.max(0, target - minSince);
                    watching = false;
                }
            }
            if (wasFeeding && !feeding) feedEnd = now;
            if (feeding) {
                feedEnd = Double.NaN;
                recoveredSince = Double.NaN;
            }
            else if (Math.abs(error) < recoveredBand) {
                if (Double.isNaN(recoveredSince)) recoveredSince = now;
            }
            else recoveredSince = Double.NaN;
            boolean recovered = !Double.isNaN(recoveredSince) && now - recoveredSince >= recoveredTime && !watching;
            boolean stale = !Double.isNaN(feedEnd) && now - feedEnd > 3;
            if (!feeding && (recovered || stale) || target <= 0) {
                if (contacts > 0 && target > 0) {
                    double mean = contactSum / contacts, spread = Math.sqrt(Math.max(0, contactSq / contacts - mean * mean));
                    double recovery = Double.isNaN(feedEnd) ? 0 : (recovered ? recoveredSince : now) - feedEnd;
                    lastShotCost = Math.abs(mean) + spread + dipWeight * dipSum / contacts + recoveryWeight * Math.max(0, recovery);
                    bursts++;
                    finished |= shots.submit(lastShotCost);
                }
                inBurst = false;
            }
        }
        wasFeeding = feeding;
        lastVelocity = velocity;
        return finished;
    }

    private void resetSteady() {

        steadyStart = Double.NaN;
        sumE2 = sumJ2 = steadyN = 0;
    }
}
