package org.firstinspires.ftc.teamcode.Systems.Turret;

//model based: servo = speed per volt (gain) + lag (timeConstant) + dead time + friction, gains come from bandwidth
public class TurretController {

    public double gain = 1800, timeConstant = 0.05, kineticFriction = 0.5, breakawayFriction = 0.7, deadTime = 0.04;
    public double bandwidth = 4, damping = 0.9, integralRatio = 0.3, frictionScale = 1;
    public double maxVelocityFraction = 0.8, integralZone = 200, maxIntegralVolts = 1.5, frictionErrorWidth = 35, movingVelocity = 70, holdTolerance = 10, referenceRate = 15;

    public boolean vibrationRejection = true;
    public double vibrationReference = 17, vibrationDeadband = 7, minimumBandwidthFraction = 0.5, minimumBandwidth = 7; //ticks; shake past the deadband by vibrationReference halves the stiffness

    public LanyardMap lanyards; //null = no lanyard compensation
    public boolean lanyardLearning = false;
    public double lanyardHome = 0, lanyardTicksPerDegree = 1, lanyardLearnRate = 3, lanyardLearnBand = 50, lanyardMaxVolts = 6, lanyardAverageTime = 0.25, lanyardStuckBand = 25; //learn rate = share of the feedback moved into the map per second

    //what the servos can really do: they run out of torque long before they run out of volts
    public double maxSpeed = Double.POSITIVE_INFINITY, maxAcceleration = Double.POSITIVE_INFINITY; //ticks/s, ticks/s^2
    //these servos don't brake when the command drops, they coast at this (ticks/s^2); they only brake when reversed
    public double maxDeceleration = Double.POSITIVE_INFINITY;
    //from rest it takes longer to get going than to change speed while moving (backlash, deadband): NaN = same as deadTime
    public double startDelay = Double.NaN;
    //ticks. The plan never heads for an end faster than it can still stop before it, whatever the target does
    public double travelMin = Double.NEGATIVE_INFINITY, travelMax = Double.POSITIVE_INFINITY;
    //the target rate jitters (Pinpoint heading rate, ~1 deg/s with the robot still). Within rateDeadband x that noise (at least
    //rateDeadbandFloor x movingVelocity) it counts as still, fading out by twice that. The plan has to pass planStartFactor x
    //movingVelocity before it counts as moving, so noise can't flip the friction push loop to loop. The noise is only measured
    //while the rate stays under rateNoiseBand x movingVelocity: real turning jerks around and would count as noise
    public double rateDeadband = 4, rateDeadbandFloor = 2, rateNoiseTime = 1, rateNoiseBand = 10, planStartFactor = 4;
    //holding, it lets go early only when it comes in faster than overpushFactor x what the loop asks for (+ overpushMargin x
    //movingVelocity): the friction guess is too high then
    public double overpushFactor = 3, overpushMargin = 3;
    //how hard it pushes while holding, learned: running past the target under its own push takes overshootCut off, and it creeps
    //back to 1 over holdRecoverTime s (the robot's turret slid on 0.65 V where STATIONARY said 1.01). Kicks still start it
    public double holdScale = 1, minHoldScale = 0.4, overshootCut = 0.1, holdRecoverTime = 20;

    public double kp, kd, ki;
    public double p, i, d, f, v, volts, lanyard;
    public double vibration, effectiveBandwidth;

    //plan is where the turret should be one lead time from now, reference is the plan from one lead time ago
    private double plan, planVelocity;
    private double reference, referenceVelocity, integral, time, loopTime = Double.NaN;
    private double lastTargetVelocity, targetAcceleration;
    private double lastError, lastError2, vibrationSquare;
    private double lastKick = -1e9, kickUntil, kickBoost, lastGap;
    private int kickLoops = 1;
    private boolean movedSinceKick = true, planWasMoving;
    private double lastRate, lastRate2, rateNoise, lastHoldError;
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

    //pole placement for gain / (s (timeConstant s + 1)): (s + a w)(s^2 + 2 z w s + w^2)
    private void gains(double w) {

        double a = integralRatio, z = damping, tau = timeConstant, k = gain;
        effectiveBandwidth = w;
        kp = tau * (1 + 2 * z * a) * w * w / k;
        kd = Math.max(0, (tau * (2 * z + a) * w - 1) / k);
        ki = tau * a * w * w * w / k;
        //a quick servo makes that ask for negative kd below ~1/tau and the loop goes limp: then it's just an integrator, cross over at w
        kp = Math.max(kp, w / k);
        ki = Math.max(ki, a * w * w / k);
    }

    /// 0 or less = not measured yet
    public void setCapability(double maxSpeed, double maxAcceleration) {
        setCapability(maxSpeed, maxAcceleration, 0);
    }

    /// 0 or less = not measured yet (deceleration then = acceleration)
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

    public void reset(double position) {

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

        //noise from the rate's second difference, a single jerk clipped. White noise: mean |that| = 1.95 sigma
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
        double lead = deadTime + loopTime; //a command lands after the dead time and is held a whole loop

        double headroom = Math.max(availableVolts - kineticFriction, 0.5);
        double maxVelocity = maxVelocityFraction * Math.min(gain * headroom, maxSpeed);

        //biggest plan speed change per loop: half the volts, or what the torque allows. Slowing down it's what coasting gives
        double step = Math.min(0.5 * gain * headroom * (1 - decay), maxVelocityFraction * maxAcceleration * loopTime);
        double slowStep = Math.min(step, maxVelocityFraction * maxDeceleration * loopTime);

        if (updates == 1) lastTargetVelocity = targetVelocity;
        targetAcceleration += ((targetVelocity - lastTargetVelocity) / dt - targetAcceleration) * Math.min(1, dt / 0.05);
        targetAcceleration = clamp(targetAcceleration, -step / loopTime, step / loopTime);
        lastTargetVelocity = targetVelocity;

        double aim = target + targetVelocity * lead + 0.5 * targetAcceleration * lead * lead;
        double aimVelocity = targetVelocity + targetAcceleration * lead;
        plan += planVelocity * dt;
        double gap = aim - plan;
        //a long loop can carry the plan past a still aim; it stops there instead of swinging back (that flips which way it pushes)
        if (gap * lastGap < 0 && Math.abs(aimVelocity) < slowStep) {
            plan = aim;
            planVelocity = aimVelocity;
            gap = 0;
        }
        lastGap = gap;
        double stopping = slowStep * (Math.sqrt(0.25 + 2 * Math.abs(gap) / (slowStep * loopTime)) - 0.5); //fastest speed that still stops in whole loops
        double closing = Math.signum(gap) * Math.min(maxVelocity, Math.min(stopping, referenceRate * Math.abs(gap)));
        double wanted = clamp(aimVelocity + closing, -maxVelocity, maxVelocity);
        //a target faster than the turret, coming round from behind, would have the plan keep going and run it off the end
        wanted = clamp(wanted, -stoppingSpeed(plan - travelMin, slowStep), stoppingSpeed(travelMax - plan, slowStep));
        if (updates == 1) wanted = 0; //first dt is junk
        double limit = (wanted - planVelocity) * planVelocity < 0 ? slowStep : step;
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

        //shake = what the error does loop to loop (second difference: slow tracking error barely moves it);
        //a stiff loop chases shake and shakes more, so stiffness drops while it shakes
        //(filtering the error instead adds lag and hunts on the friction)
        double jitter = updates > 2 ? (error - 2 * lastError + lastError2) / Math.sqrt(6) : 0;
        lastError2 = lastError;
        lastError = error;
        if (updates > 2 && Math.abs(change) < 0.25 * step) vibrationSquare += (jitter * jitter - vibrationSquare) * Math.min(1, dt / 0.5); //not while the plan itself accelerates
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

        //exact inverse of the lag over one held loop, (v + tau * a) / gain overdrives on slow loops
        v = (planVelocity - decay * startVelocity) / ((1 - decay) * gain);
        //stuck: a short kick at breakaway, then kinetic until a start delay shows whether it went. Holding breakaway until the
        //velocity shows motion hops way past small targets. A kick that didn't move it comes back longer and harder (so a friction
        //guess that's too low can't leave it stuck), one that did comes back a bit shorter: it settles on what this servo needs
        boolean stopped = Math.abs(velocity) < movingVelocity;
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
            kickUntil = time + (kickLoops - 0.5) * loopTime;
            movedSinceKick = false;
            kicking = true;
        }
        double breakaway = breakawayFriction + kickBoost;
        //run past the reference: reverse past friction, that's the only way these servos brake (easing off only coasts)
        double pushing = Math.signum(planVelocity);
        if (error * pushing < -2 * frictionErrorWidth) pushing = -pushing;
        if (planMoving) f = (kicking ? breakaway : kineticFriction) * pushing;
        else if (holding) f = 0;
        else if (kicking) f = breakaway * frictionScale * Math.signum(error);
        else {
            //it slides on less than it takes to start, so the friction guess is often high and pushing all the way in runs it past
            //and back, forever. Coming in that fast it lets go once it gets there within a lead anyway, well past it pushes back to brake
            double toward = Math.max(0, velocity * Math.signum(error));
            boolean overpushed = toward > overpushFactor * effectiveBandwidth * Math.abs(error) + overpushMargin * movingVelocity;
            double left = Math.abs(error) - (overpushed ? toward * lead : 0);
            f = kineticFriction * frictionScale * holdScale * clamp((left - holdTolerance) / frictionErrorWidth, left < -2 * frictionErrorWidth ? -1 : 0, 1) * Math.signum(error);
        }
        //ran past the target under its own push: it pushes too hard
        if (!planMoving && error * lastHoldError < 0 && Math.abs(velocity) > overpushMargin * movingVelocity && Math.abs(lastHoldError) > holdTolerance) holdScale = Math.max(minHoldScale, holdScale * (1 - overshootCut));
        lastHoldError = planMoving ? 0 : error;
        holdScale += (1 - holdScale) * Math.min(1, dt / holdRecoverTime);
        p = kp * error;
        d = kd * velocityError;

        //lanyard pull where the turret will be when this command lands
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

        //learns while the plan glides (steady speed, not starts and stops) into that direction's table. It doesn't wait for the turret
        //to move smoothly: going with a lanyard it stick-slips until the map has learned to brake it.
        //Also learns while stuck off target at a hold, into both tables, which is how holds near a lanyard get exact
        boolean gliding = planMoving && Math.abs(change) < 0.25 * step;
        boolean stuck = !planMoving && !holding && Math.abs(referenceVelocity) < movingVelocity && Math.abs(velocity) < 5 * movingVelocity; //it creeps while stuck
        boolean learning = (gliding && Math.abs(error) < lanyardLearnBand || stuck && Math.abs(error) < lanyardStuckBand) && !saturated; //tight band when stuck, a jam shouldn't teach it much
        //it learns the average over lanyardAverageTime at the average spot, so stick-slip doesn't put spikes in the map
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
                if (ki > 0) integral -= share * i / ki; //hand the integral's part to the map so it isn't counted twice
                i = ki * integral;
            }
        }

        volts = clamp(p + i + d + f + v + lanyard, -availableVolts, availableVolts);
        return volts;
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

    /// s, averaged. NaN before the first update
    public double getLoopTime() {
        return loopTime;
    }

    //fastest speed that still stops within distance, slowing by step a loop
    private double stoppingSpeed(double distance, double step) {

        if (Double.isInfinite(distance)) return Double.POSITIVE_INFINITY;
        if (!(distance > 0)) return 0;
        return step * (Math.sqrt(0.25 + 2 * distance / (step * loopTime)) - 0.5);
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
