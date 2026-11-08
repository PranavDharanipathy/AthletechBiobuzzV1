package org.firstinspires.ftc.teamcode.Systems.Turret;

public class TurretController {

    public double gain = 1800, timeConstant = 0.05, kineticFriction = 0.5, breakawayFriction = 0.7, deadTime = 0.04;
    public double bandwidth = 4, damping = 0.9, integralRatio = 0.3, frictionScale = 1;
    public double maxVelocityFraction = 0.8, integralZone = 200, maxIntegralVolts = 1.5, frictionErrorWidth = 35, movingVelocity = 70, holdTolerance = 10, referenceRate = 15;

    public boolean vibrationRejection = true;
    public double vibrationReference = 17, vibrationDeadband = 7, minimumBandwidthFraction = 0.5, minimumBandwidth = 7;

    public LanyardMap lanyards;
    public boolean lanyardLearning = false;
    public double lanyardHome = 0, lanyardTicksPerDegree = 1, lanyardLearnRate = 3, lanyardLearnBand = 50, lanyardMaxVolts = 6, lanyardAverageTime = 0.25, lanyardStuckBand = 25;

    public double maxSpeed = Double.POSITIVE_INFINITY, maxAcceleration = Double.POSITIVE_INFINITY;
    //they coast when the command drops, only brake when reversed
    public double maxDeceleration = Double.POSITIVE_INFINITY;
    public double decelerationFraction = 0.8;
    public double startDelay = Double.NaN;
    public double travelMin = Double.NEGATIVE_INFINITY, travelMax = Double.POSITIVE_INFINITY;
    public double rateDeadband = 4, rateDeadbandFloor = 2, rateNoiseTime = 1, rateNoiseBand = 10, planStartFactor = 4;
    public double overpushFactor = 3, overpushMargin = 3;
    public double holdScale = 1, minHoldScale = 0.4, overshootCut = 0.1, holdRecoverTime = 20;
    public double stuckEndZone = 60 * 68.27, stuckRate = 2, stuckMax = 1.5, stuckWait = 0.2, stuckRelease = 0.2, stuckVoltsLimit = 2.9;
    public double resyncBand = 500;
    public boolean resyncStopsAtTarget = true;
    public double accelerationFraction = 0.8;
    public double brakeBand = 0;
    public int resyncs;
    public boolean landingEnabled = false, brakeLearning = true;
    public double landingMinDistance = 680, landingMinSpeed = 2000, landingAimRate = 700, landingEntry = 0.7;
    public double landingExitDistance = 70, landingExitSpeed = 1400, landingStep = 0.003, landingHorizon = 0.6;
    public double brakeScale = 1, minBrakeScale = 0.5, maxBrakeScale = 1.6, brakeLearnRate = 0.5;
    public double landingBias = 0, maxLandingBias = 700, biasLearnRate = 0.5;
    public boolean landing;
    public double landingVolts, lastLandingOvershoot;
    public int landings;
    private double landingTarget, landingDir, landingStart, landingEntrySpeed, predictedSlowing, sentVolts, lastLandingTime;
    private boolean watchingRest;

    public double kp, kd, ki;
    public double p, i, d, f, v, volts, lanyard;
    public double vibration, effectiveBandwidth;

    private double plan, planVelocity;
    private double reference, referenceVelocity, integral, time, loopTime = Double.NaN;
    private double lastTargetVelocity, targetAcceleration;
    private double lastError, lastError2, vibrationSquare;
    private double lastKick = -1e9, kickUntil, kickBoost, lastGap;
    private int kickLoops = 1;
    private boolean movedSinceKick = true, planWasMoving;
    private double lastRate, lastRate2, rateNoise, lastHoldError, lastPosition, lastAway, stuckVolts, stuckSince = Double.NaN;
    private double learnDirection = Double.NaN, learnTime, learnLast, learnResidual, learnDegrees;
    private int updates = 0;
    private boolean started = false;

    private static final int HISTORY = 64;
    private final double[] historyTime = new double[HISTORY], historyPosition = new double[HISTORY], historyVelocity = new double[HISTORY];
    private int historyNewest = -1, historyCount = 0;

    public void configure(double gain, double timeConstant, double kineticFriction, double breakawayFriction, double deadTime, double bandwidth, double damping, double integralRatio, double frictionScale) {

        this.gain = Math.max(gain, 1);
        this.timeConstant = Math.max(timeConstant, 0.001);
        this.kineticFriction = Math.max(kineticFriction, 0);
        this.breakawayFriction = Math.max(breakawayFriction, 0);
        this.deadTime = Math.max(deadTime, 0);
        this.bandwidth = Math.max(bandwidth, 0.1);
        this.damping = damping;
        this.integralRatio = Math.max(integralRatio, 0);
        this.frictionScale = frictionScale;

        gains(this.bandwidth);
    }

    private void gains(double w) {

        double a = integralRatio, z = damping, tau = timeConstant, k = gain;
        effectiveBandwidth = w;
        kp = tau * (1 + 2 * z * a) * w * w / k;
        kd = Math.max(0, (tau * (2 * z + a) * w - 1) / k);
        ki = tau * a * w * w * w / k;
        //quick servo: negative kd below ~1/tau, use an integrator
        kp = Math.max(kp, w / k);
        ki = Math.max(ki, a * w * w / k);
    }

    public void setCapability(double maxSpeed, double maxAcceleration) {
        setCapability(maxSpeed, maxAcceleration, 0);
    }

    public void setCapability(double maxSpeed, double maxAcceleration, double maxDeceleration) {

        this.maxSpeed = maxSpeed > 0 ? maxSpeed : Double.POSITIVE_INFINITY;
        this.maxAcceleration = maxAcceleration > 0 ? maxAcceleration : Double.POSITIVE_INFINITY;
        this.maxDeceleration = maxDeceleration > 0 ? maxDeceleration : this.maxAcceleration;
    }

    public void setTravel(double min, double max) {

        travelMin = Math.min(min, max);
        travelMax = Math.max(min, max);
    }

    public void setLimits(double maxVelocityFraction, double integralZone, double maxIntegralVolts, double frictionErrorWidth, double movingVelocity, double holdTolerance) {

        this.holdTolerance = holdTolerance;
        this.maxVelocityFraction = maxVelocityFraction;
        this.integralZone = integralZone;
        this.maxIntegralVolts = maxIntegralVolts;
        this.frictionErrorWidth = Math.max(frictionErrorWidth, 1e-6);
        this.movingVelocity = movingVelocity;
    }

    public double getRateNoise() {
        return rateNoise;
    }

    public double getKickBoost() {
        return kickBoost;
    }

    public double getHoldScale() {
        return holdScale;
    }

    public void reset(double position) {

        lastAway = stuckVolts = 0;
        stuckSince = Double.NaN;
        plan = reference = position;
        planVelocity = referenceVelocity = 0;
        integral = 0;
        time = 0;
        loopTime = Double.NaN;
        updates = 0;
        lastTargetVelocity = targetAcceleration = 0;
        lastError = lastError2 = vibrationSquare = vibration = 0;
        learnTime = learnLast = learnResidual = learnDegrees = 0;
        lastKick = -1e9;
        kickUntil = kickBoost = lastGap = 0;
        kickLoops = 1;
        movedSinceKick = true;
        planWasMoving = false;
        lastRate = lastRate2 = rateNoise = lastHoldError = 0;
        resyncs = 0;
        landing = false;
        sentVolts = 0;
        learnDirection = Double.NaN;
        historyNewest = -1;
        historyCount = 0;
        started = true;
    }

    public double update(double dt, double target, double targetVelocity, double position, double velocity, double availableVolts) {

        if (!started) reset(position);
        if (!(dt > 0) || dt > 0.25) dt = 0.01;
        if (Double.isNaN(target) || Double.isNaN(position)) return volts = 0;
        if (Double.isNaN(targetVelocity)) targetVelocity = 0;
        if (Double.isNaN(velocity)) velocity = 0;

        double rateJerk = Math.abs(targetVelocity - 2 * lastRate + lastRate2);
        boolean slow = Math.max(Math.abs(targetVelocity), Math.max(Math.abs(lastRate), Math.abs(lastRate2))) < rateNoiseBand * movingVelocity;
        if (updates >= 2 && slow) rateNoise += (Math.min(rateJerk, 3 * rateNoise + movingVelocity) - rateNoise) * Math.min(1, dt / rateNoiseTime);
        lastRate2 = lastRate;
        lastRate = targetVelocity;
        double deadband = Math.max(rateDeadbandFloor * movingVelocity, rateDeadband * rateNoise / 1.954);
        if (deadband > 0) targetVelocity = Math.signum(targetVelocity) * Math.max(0, Math.abs(targetVelocity) - deadband * clamp(2 - Math.abs(targetVelocity) / deadband, 0, 1));

        time += dt;
        loopTime = updates++ < 3 ? dt : loopTime + 0.3 * (dt - loopTime);
        double decay = Math.exp(-loopTime / timeConstant);
        double lead = deadTime + loopTime;

        double headroom = Math.max(availableVolts - kineticFriction, 0.5);
        double maxVelocity = maxVelocityFraction * Math.min(gain * headroom, maxSpeed);

        double step = Math.min(0.5 * gain * headroom * (1 - decay), accelerationFraction * maxAcceleration * loopTime);
        double slowStep = Math.min(step, decelerationFraction * maxDeceleration * loopTime);

        if (updates > 3) {
            lookBack(time - lead);
            if (Math.abs(reference - position) > resyncBand) resync(position, velocity, maxVelocity, target, slowStep);
        }

        if (updates == 1) lastTargetVelocity = targetVelocity;
        targetAcceleration += ((targetVelocity - lastTargetVelocity) / dt - targetAcceleration) * Math.min(1, dt / 0.05);
        targetAcceleration = clamp(targetAcceleration, -step / loopTime, step / loopTime);
        lastTargetVelocity = targetVelocity;

        double aim = target + targetVelocity * lead + 0.5 * targetAcceleration * lead * lead;
        double aimVelocity = targetVelocity + targetAcceleration * lead;
        if (land(target, aimVelocity, position, velocity, availableVolts, lead)) return volts;
        double closeStep = slowStep;
        if (landingEnabled && !Double.isInfinite(maxDeceleration) && Math.abs(aimVelocity) < landingAimRate) closeStep = Math.max(slowStep, -landingEntry * acceleration(0.5 * maxVelocity, -landingEntry * availableVolts) * loopTime);
        plan += planVelocity * dt;
        double gap = aim - plan;
        if (gap * lastGap < 0 && Math.abs(aimVelocity) < slowStep) {
            plan = aim;
            planVelocity = aimVelocity;
            gap = 0;
        }
        lastGap = gap;
        double stopping = closeStep * (Math.sqrt(0.25 + 2 * Math.abs(gap) / (closeStep * loopTime)) - 0.5);
        double closing = Math.signum(gap) * Math.min(maxVelocity, Math.min(stopping, referenceRate * Math.abs(gap)));
        double wanted = clamp(aimVelocity + closing, -maxVelocity, maxVelocity);
        wanted = clamp(wanted, -stoppingSpeed(plan - travelMin, slowStep), stoppingSpeed(travelMax - plan, slowStep));
        if (updates == 1) wanted = 0; //first dt is junk
        double limit = (wanted - planVelocity) * planVelocity < 0 ? closeStep : step;
        double change = clamp(wanted - planVelocity, -limit, limit);
        double startVelocity = planVelocity;
        planVelocity += change;
        if (Math.abs(aim - plan) < 1 && Math.abs(aimVelocity - planVelocity) < step) {
            plan = aim;
            planVelocity = aimVelocity;
        }

        remember(plan, planVelocity);
        lookBack(time - lead);

        double error = reference - position;
        double velocityError = referenceVelocity - velocity;

        double jitter = updates > 2 ? (error - 2 * lastError + lastError2) / Math.sqrt(6) : 0;
        lastError2 = lastError;
        lastError = error;
        if (updates > 2 && Math.abs(change) < 0.25 * step) vibrationSquare += (jitter * jitter - vibrationSquare) * Math.min(1, dt / 0.5);
        vibration = Math.sqrt(vibrationSquare);
        if (vibrationRejection) {
            double ratio = Math.max(0, vibration - vibrationDeadband) / Math.max(vibrationReference, 1e-6);
            double floor = Math.min(bandwidth, Math.max(minimumBandwidth, minimumBandwidthFraction * bandwidth));
            gains(Math.max(floor, bandwidth / (1 + ratio * ratio)));
        }
        else if (effectiveBandwidth != bandwidth) gains(bandwidth);

        boolean planMoving = Math.abs(planVelocity) > (planWasMoving ? 1 : planStartFactor) * movingVelocity;
        planWasMoving = planMoving;
        boolean holding = !planMoving && Math.abs(referenceVelocity) < planStartFactor * movingVelocity && Math.abs(error) < holdTolerance;

        v = (planVelocity - decay * startVelocity) / ((1 - decay) * gain);
        boolean stopped = Math.abs(velocity) < movingVelocity || updates > 1 && Math.abs(error) > integralZone && Math.abs(position - lastPosition) < movingVelocity * dt;
        double startWait = (startDelay > 0 ? Math.max(startDelay, deadTime) : deadTime) + loopTime;
        if (!stopped) {
            if (!movedSinceKick && lastKick > -1e8) {
                kickLoops = Math.max(1, kickLoops - 1);
                kickBoost *= 0.5;
            }
            movedSinceKick = true;
            kickUntil = 0;
        }
        if (holding) movedSinceKick = true;
        boolean kicking = time < kickUntil;
        if (stopped && !kicking && !holding && time - lastKick >= startWait + kickLoops * loopTime) {
            if (!movedSinceKick && lastKick > -1e8) {
                kickLoops = Math.min(2 * kickLoops, Math.max(1, (int) Math.ceil(startWait / loopTime)));
                kickBoost = Math.min(kickBoost + 0.08, 0.6);
            }
            lastKick = time;
            int loops = Math.abs(error) > integralZone ? Math.max(kickLoops, (int) Math.ceil(startWait / loopTime)) : kickLoops;
            kickUntil = time + (loops - 0.5) * loopTime;
            movedSinceKick = false;
            kicking = true;
        }
        double breakaway = breakawayFriction + kickBoost;
        double pushing = Math.signum(planVelocity);
        boolean slowing = change * planVelocity < 0;
        if (brakeBand > 0 && !slowing) pushing *= clamp(1 + 2 * Math.min(0, error * pushing) / brakeBand, -1, 1);
        else if (error * pushing < -2 * frictionErrorWidth) pushing = -pushing;
        if (planMoving) f = (kicking ? breakaway : kineticFriction) * pushing;
        else if (holding) f = 0;
        else if (kicking) f = breakaway * frictionScale * Math.signum(error);
        else {
            //it slides on less than it takes to start, lets go early
            double toward = Math.max(0, velocity * Math.signum(error));
            boolean overpushed = toward > overpushFactor * effectiveBandwidth * Math.abs(error) + overpushMargin * movingVelocity;
            double left = Math.abs(error) - (overpushed ? toward * lead : 0);
            double scale = holdScale + (1 - holdScale) * clamp((Math.abs(error) - integralZone) / Math.max(integralZone, 1e-6), 0, 1);
            f = kineticFriction * frictionScale * scale * clamp((left - holdTolerance) / frictionErrorWidth, left < -2 * frictionErrorWidth ? -1 : 0, 1) * Math.signum(error);
        }
        if (!planMoving && error * lastHoldError < 0 && Math.abs(velocity) > overpushMargin * movingVelocity && Math.abs(lastHoldError) > holdTolerance) holdScale = Math.max(minHoldScale, holdScale * (1 - overshootCut));
        lastHoldError = planMoving ? 0 : error;
        holdScale += (1 - holdScale) * Math.min(1, dt / holdRecoverTime);
        p = kp * error;
        d = kd * velocityError;

        double direction = planMoving ? Math.signum(planVelocity) : 0;
        double lanyardDegrees = (position + velocity * lead - lanyardHome) / lanyardTicksPerDegree;
        lanyard = lanyards != null ? lanyards.volts(lanyardDegrees, direction) : 0;

        double unclamped = p + ki * integral + d + f + v + lanyard;
        boolean saturated = Math.abs(unclamped) >= availableVolts;
        boolean pushingFurther = Math.signum(unclamped) == Math.signum(error);
        if (!holding && Math.abs(error) < integralZone && !(saturated && pushingFurther)) integral += error * dt;
        if (Math.abs(error) >= integralZone) integral *= 0.9;
        if (ki > 0) integral = clamp(integral, -maxIntegralVolts / ki, maxIntegralVolts / ki);
        i = ki * integral;

        boolean gliding = planMoving && Math.abs(change) < 0.25 * step;
        boolean stuck = !planMoving && !holding && Math.abs(referenceVelocity) < movingVelocity && Math.abs(velocity) < 5 * movingVelocity;
        boolean learning = (gliding && Math.abs(error) < lanyardLearnBand || stuck && Math.abs(error) < lanyardStuckBand) && !saturated;
        if (lanyards != null && lanyardLearning && learning) {
            if (learnDirection != direction || time - learnLast > lanyardAverageTime) {
                learnDirection = direction;
                learnTime = 0;
                learnResidual = p + i;
                learnDegrees = lanyardDegrees;
            }
            double a = Math.min(1, dt / Math.max(lanyardAverageTime, dt));
            learnResidual += (p + i - learnResidual) * a;
            learnDegrees += (lanyardDegrees - learnDegrees) * a;
            learnTime += dt;
            learnLast = time;
            if (learnTime >= lanyardAverageTime) {
                double share = Math.min(1, lanyardLearnRate * dt);
                lanyards.learn(learnDegrees, direction, share * learnResidual, lanyardMaxVolts);
                if (ki > 0) integral -= share * i / ki;
                i = ki * integral;
            }
        }

        lastPosition = position;
        double away = target - position;
        if (away * lastAway < 0) stuckVolts = 0;
        lastAway = away;
        double toward = velocity * Math.signum(away);
        boolean nearEnd = target > travelMax - stuckEndZone || target < travelMin + stuckEndZone;
        boolean noProgress = nearEnd && Math.abs(away) > holdTolerance && Math.abs(aimVelocity) < 4 * movingVelocity && toward < Math.max(movingVelocity, Math.abs(away));
        if (!noProgress) stuckSince = Double.NaN;
        else if (Double.isNaN(stuckSince)) stuckSince = time;
        if (noProgress && time - stuckSince >= stuckWait) stuckVolts = clamp(stuckVolts + Math.signum(away) * stuckRate * dt, -stuckMax, stuckMax);
        else if (toward > Math.max(4 * movingVelocity, 2 * Math.abs(away))) stuckVolts *= Math.exp(-dt / Math.max(stuckRelease, 1e-6));
        double base = p + i + d + f + v + lanyard;
        double push = stuckVolts > 0 ? clamp(Math.min(base + stuckVolts, stuckVoltsLimit) - base, 0, stuckVolts) : clamp(Math.max(base + stuckVolts, -stuckVoltsLimit) - base, stuckVolts, 0);
        volts = clamp(base + push, -availableVolts, availableVolts);
        sentVolts = volts;
        return volts;
    }

    public double getStuckVolts() {
        return stuckVolts;
    }

    private void resync(double position, double velocity, double maxVelocity, double target, double slowStep) {

        double offset = position - reference;
        if (offset * planVelocity >= 0) {
            double was = planVelocity;
            double gap = target - position;
            double stop = stoppingSpeed(Math.abs(gap), slowStep);
            planVelocity = clamp(velocity, -maxVelocity, maxVelocity);
            if (resyncStopsAtTarget) planVelocity = gap >= 0 ? clamp(planVelocity, 0, stop) : clamp(planVelocity, -stop, 0);
            for (int k = 0; k < HISTORY; k++) historyVelocity[k] += planVelocity - was;
        }
        offset = clamp(plan + offset, travelMin, travelMax) - plan;
        plan += offset;
        for (int k = 0; k < HISTORY; k++) historyPosition[k] += offset;
        reference = position;
        integral = 0;
        lastGap = 0;
        lastError = lastError2 = lastHoldError = 0;
        resyncs++;
    }

    private boolean land(double target, double aimVelocity, double position, double velocity, double availableVolts, double lead) {

        boolean usable = landingEnabled && !Double.isInfinite(maxDeceleration) && updates > 3;
        if (watchingRest && !landing) {
            if (Math.abs(target - landingTarget) > landingExitDistance) watchingRest = false;
            else if (velocity * landingDir <= 0) {
                lastLandingOvershoot = (position - landingTarget) * landingDir;
                if (brakeLearning) landingBias = clamp(landingBias + biasLearnRate * lastLandingOvershoot, -maxLandingBias, maxLandingBias);
                watchingRest = false;
            }
        }
        if (!landing) {
            double toward = (target - position) * Math.signum(velocity);
            if (!usable || Math.abs(aimVelocity) > landingAimRate || Math.abs(velocity) < landingMinSpeed || toward < landingMinDistance) return false;
            if (stopAt(Math.abs(velocity), sentVolts * Math.signum(velocity), -landingEntry * availableVolts) < toward) return false;
            landing = true;
            landings++;
            landingTarget = target;
            landingDir = Math.signum(velocity);
            landingStart = lastLandingTime = time;
            landingEntrySpeed = Math.abs(velocity);
            predictedSlowing = 0;
        }
        double toward = (landingTarget - position) * landingDir - landingBias;
        double speed = velocity * landingDir;
        predictedSlowing += -acceleration(Math.max(speed, 0), sentVolts * landingDir) * (time - lastLandingTime);
        lastLandingTime = time;
        boolean done = speed < landingExitSpeed || toward < landingExitDistance || time - landingStart > landingHorizon;
        if (!usable || done || Math.abs(target - landingTarget) > landingExitDistance || Math.abs(aimVelocity) > 2 * landingAimRate) {
            double slowed = landingEntrySpeed - Math.max(speed, 0);
            if (done && brakeLearning && usable && predictedSlowing > 0.2 * landingEntrySpeed && slowed > 0) {
                brakeScale = clamp(brakeScale * Math.pow(slowed / predictedSlowing, brakeLearnRate), minBrakeScale, maxBrakeScale);
            }
            landing = false;
            watchingRest = done;
            plan = position + velocity * lead;
            planVelocity = velocity;
            historyNewest = -1;
            historyCount = 0;
            lastGap = 0;
            lastError = lastError2 = lastHoldError = 0;
            return false;
        }
        double u = landingVolts(speed, toward, sentVolts * landingDir, availableVolts);
        landingVolts = u;
        volts = clamp(landingVolts * landingDir, -availableVolts, availableVolts);
        p = i = d = v = lanyard = 0;
        f = volts;
        sentVolts = volts;
        plan = position + velocity * lead;
        planVelocity = velocity;
        historyNewest = -1;
        historyCount = 0;
        remember(plan, planVelocity);
        reference = position;
        referenceVelocity = velocity;
        lastPosition = position;
        return true;
    }

    private double landingVolts(double speed, double distance, double sending, double availableVolts) {

        if (stopAt(speed, sending, 0) <= distance) return 0;
        if (stopAt(speed, sending, -availableVolts) >= distance) return -availableVolts;
        double lo = -availableVolts, hi = 0;
        for (int k = 0; k < 14; k++) {
            double mid = 0.5 * (lo + hi);
            if (stopAt(speed, sending, mid) > distance) hi = mid;
            else lo = mid;
        }
        return 0.5 * (lo + hi);
    }

    private double stopAt(double speed, double sending, double u) {

        double x = 0, w = speed, t = 0, h = landingStep;
        while (w > 0 && t < landingHorizon) {
            double command = t < deadTime ? sending : u;
            w += acceleration(w, command) * h;
            x += Math.max(w, 0) * h;
            t += h;
        }
        return x;
    }

    private double acceleration(double w, double u) {

        double coast = maxDeceleration * brakeScale;
        if (u > kineticFriction) {
            double target = gain * (u - kineticFriction);
            if (target >= w) return Math.min((target - w) / timeConstant, maxAcceleration);
            return -Math.min(coast, (w - target) / timeConstant);
        }
        if (u >= -kineticFriction) return -coast;
        double target = gain * (u + kineticFriction);
        return -Math.min(Math.max(coast, (w - target) / timeConstant * brakeScale), maxAcceleration * brakeScale + coast);
    }

    private void remember(double position, double velocity) {

        historyNewest = (historyNewest + 1) % HISTORY;
        historyTime[historyNewest] = time;
        historyPosition[historyNewest] = position;
        historyVelocity[historyNewest] = velocity;
        if (historyCount < HISTORY) historyCount++;
    }

    private void lookBack(double when) {

        int newer = historyNewest;
        for (int k = 1; k < historyCount; k++) {
            int older = (historyNewest - k + HISTORY) % HISTORY;
            if (historyTime[older] <= when) {
                double span = historyTime[newer] - historyTime[older];
                double s = span > 0 ? (when - historyTime[older]) / span : 0;
                reference = historyPosition[older] + s * (historyPosition[newer] - historyPosition[older]);
                referenceVelocity = historyVelocity[older] + s * (historyVelocity[newer] - historyVelocity[older]);
                return;
            }
            newer = older;
        }
        reference = historyPosition[newer];
        referenceVelocity = historyVelocity[newer];
    }

    public double getReference() {
        return reference;
    }

    public double getReferenceVelocity() {
        return referenceVelocity;
    }

    public double getPlannedVelocity() {
        return planVelocity;
    }

    public double getLoopTime() {
        return loopTime;
    }

    private double stoppingSpeed(double distance, double step) {

        if (Double.isInfinite(distance)) return Double.POSITIVE_INFINITY;
        if (!(distance > 0)) return 0;
        return step * (Math.sqrt(0.25 + 2 * distance / (step * loopTime)) - 0.5);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
