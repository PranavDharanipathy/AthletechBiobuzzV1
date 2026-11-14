package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelHyperTuner {

    public final OnlineTuner stability, shots;
    public double steadyBand = 40, steadyWindow = 1.5, quietAfterFeed = 0.6, recoveredBand = 30, recoveredTime = 0.15, staleSeconds = 3;
    //a ball: speed falls minDip from where it was within lookBack and ends belowTarget under the target (overshoot settling never does)
    public double minDip = 40, belowTarget = 25, lookBack = 0.35, sameBall = 0.12, afterClose = 0.8, dipEndRise = 20, settleWindow = 0.6;
    public double recoveryWeight = 300, dipWeight = 0.5, settleWeight = 1, overshootWeight = 0.5, laterBallWeight = 0.5, targetStep = 10;
    public double lastSteadyCost = Double.NaN, lastShotCost = Double.NaN, lastContactError = Double.NaN, lastRecovery = Double.NaN, lastOvershoot = Double.NaN;
    public int bursts, balls, ballsInBurst;

    private static final int HISTORY = 40;
    private final double[] historyTime = new double[HISTORY], historyVelocity = new double[HISTORY];
    private int historyCount, historyNext;

    private double steadyStart = Double.NaN, sumE2, sumJ2, steadyN, lastVelocity = Double.NaN, lastFeedEnd = -1e9, lastTarget;
    private boolean inBurst, inDip;
    private double feedEnd = Double.NaN, lastContact = -1e9, low, recoveredSince = Double.NaN, settleStart = Double.NaN, recovery;
    private double weightSum, errorSum, errorSq, dipSum, settleE2, settleN, overshoot;

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

    //velocity = the flywheel's filtered speed
    public boolean update(double now, double target, double velocity, boolean feeding) {

        boolean finished = false;
        double error = target - velocity;
        if (Math.abs(target - lastTarget) > targetStep) resetSteady();
        lastTarget = target;
        if (feeding) lastFeedEnd = now;
        remember(now, velocity);

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

        if (target <= 0) {
            inBurst = false;
            lastVelocity = velocity;
            return finished;
        }
        if (feeding && !inBurst) startBurst();
        if (inBurst) {
            if (feeding) {
                feedEnd = Double.NaN;
                settleStart = Double.NaN;
            }
            else if (Double.isNaN(feedEnd)) feedEnd = now;
            //balls still in the path land after the blocker closes
            boolean watching = feeding || now - feedEnd <= afterClose;
            if (watching) findBall(now, target, velocity);
            if (inDip) {
                low = Math.min(low, velocity);
                if (velocity > low + dipEndRise) {
                    dipSum += ballWeight(ballsInBurst) * Math.max(0, target - low);
                    inDip = false;
                }
            }
            if (!feeding && !inDip && Math.abs(error) < recoveredBand) {
                if (Double.isNaN(recoveredSince)) recoveredSince = now;
            }
            else recoveredSince = Double.NaN;
            //the wobble just after it's back counts from the moment it's back (the overshoot peaks ~0.3 s after a contact), a late ball starts it over
            if (!feeding && Double.isNaN(settleStart) && !Double.isNaN(recoveredSince) && now - recoveredSince >= recoveredTime) {
                settleStart = now;
                recovery = ballsInBurst > 0 ? Math.max(0, recoveredSince - lastContact) : 0;
                settleE2 = settleN = overshoot = 0;
            }
            if (!Double.isNaN(settleStart)) {
                if (now - settleStart < settleWindow) {
                    settleE2 += error * error;
                    settleN++;
                    overshoot = Math.max(overshoot, velocity - target);
                }
                else if (!watching) finished |= finishBurst();
            }
            else if (!feeding && now - feedEnd > staleSeconds) {
                recovery = ballsInBurst > 0 ? now - lastContact : 0;
                finished |= finishBurst();
            }
        }
        lastVelocity = velocity;
        return finished;
    }

    private void startBurst() {

        inBurst = true;
        inDip = false;
        ballsInBurst = 0;
        weightSum = errorSum = errorSq = dipSum = 0;
        settleE2 = settleN = overshoot = recovery = 0;
        feedEnd = Double.NaN;
        settleStart = Double.NaN;
        recoveredSince = Double.NaN;
    }

    private void findBall(double now, double target, double velocity) {

        if (inDip || velocity >= target - belowTarget) return;
        //walk back up the fall to where it started: that's the contact, and the speed there is the speed the ball got
        int k = newest();
        double top = velocity, topTime = now;
        for (int n = 1; n < historyCount; n++) {
            int j = (k - n + HISTORY) % HISTORY, later = (j + 1) % HISTORY;
            if (now - historyTime[j] > lookBack || historyVelocity[j] < historyVelocity[later] - 2) break;
            if (historyVelocity[j] >= top) {
                top = historyVelocity[j];
                topTime = historyTime[j];
            }
        }
        if (top - velocity < minDip) return;
        inDip = true;
        low = velocity;
        if (topTime - lastContact <= sameBall) return;
        ballsInBurst++;
        balls++;
        settleStart = Double.NaN;
        lastContact = topTime;
        double w = ballWeight(ballsInBurst), e = top - target;
        weightSum += w;
        errorSum += w * e;
        errorSq += w * e * e;
    }

    //later balls in a burst count more: match rapid fire is where recovery shows
    private double ballWeight(int ball) {
        return 1 + laterBallWeight * Math.max(0, ball - 1);
    }

    private boolean finishBurst() {

        inBurst = false;
        if (ballsInBurst == 0) return false;
        double mean = errorSum / weightSum, spread = Math.sqrt(Math.max(0, errorSq / weightSum - mean * mean));
        double settle = settleN > 0 ? Math.sqrt(settleE2 / settleN) : 0;
        lastContactError = mean;
        lastRecovery = recovery;
        lastOvershoot = overshoot;
        lastShotCost = Math.abs(mean) + spread + dipWeight * dipSum / weightSum + recoveryWeight * recovery + settleWeight * settle + overshootWeight * Math.max(0, overshoot);
        bursts++;
        return shots.submit(lastShotCost);
    }

    private void remember(double now, double velocity) {

        historyTime[historyNext] = now;
        historyVelocity[historyNext] = velocity;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    private int newest() {
        return (historyNext - 1 + HISTORY) % HISTORY;
    }

    private void resetSteady() {

        steadyStart = Double.NaN;
        sumE2 = sumJ2 = steadyN = 0;
    }
}
