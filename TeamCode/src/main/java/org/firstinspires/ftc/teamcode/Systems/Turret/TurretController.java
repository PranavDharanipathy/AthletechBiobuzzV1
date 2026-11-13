package org.firstinspires.ftc.teamcode.Systems.Turret;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;

public class TurretController {

    public double kpFar, kpClose, pSwitch;
    public double kiFar, kiClose, iCloseZone, iZone, kISmash, minI, maxI;
    public double kdFar, kdClose, dSwitch, dMin, dMax, kDFilter;
    public double kVelocityFilter;
    public double kHold, holdDecay;
    public double gain;
    public double kStatic, kCoulomb, stribeckVelocity, frictionDeadband, frictionRelease, frictionRamp;
    public boolean pulseEnabled;
    public double pulseZone, pulseAimRate, pulseOverBreakaway, pulseSeconds, pulseMin, pulseMax, pulseWait, pulseStillVelocity, pulseEnterVelocity;
    public boolean frictionFromVelocity, stribeckFromMeasured;
    public double travelMin = Double.NEGATIVE_INFINITY, travelMax = Double.POSITIVE_INFINITY, endDecel, endLag;

    public double p, i, d, f, v, s, positionalVelocity;
    public boolean pulsing;
    public double pulseLength = Double.NaN;
    //learned in the match from the pulses: battery, servo supply and friction drift; kept across reset()
    public double frictionTrim, frictionTrimStep, frictionTrimMax;
    public int pulsesWithoutCounts;

    private double integral, prevError, filteredDerivative, lastPosition, measuredVelocity;
    private boolean first = true, held, pulseOn, pulsed;
    private double clock, pulseUntil, waitUntil, pulseFrom, pulseSign, pulseError, tpd = 1;

    public void applyConstants(double tpd) {

        this.tpd = tpd;
        kpFar = TurretConstants.TURRET_KP_FAR;
        kpClose = TurretConstants.TURRET_KP_CLOSE;
        pSwitch = TurretConstants.TURRET_P_SWITCH_DEGREES * tpd;
        kiFar = TurretConstants.TURRET_KI_FAR;
        kiClose = TurretConstants.TURRET_KI_CLOSE;
        iCloseZone = TurretConstants.TURRET_I_CLOSE_DEGREES * tpd;
        iZone = TurretConstants.TURRET_I_ZONE_DEGREES * tpd;
        kISmash = TurretConstants.TURRET_KI_SMASH;
        minI = -TurretConstants.TURRET_MAX_I;
        maxI = TurretConstants.TURRET_MAX_I;
        kdFar = TurretConstants.TURRET_KD_FAR;
        kdClose = TurretConstants.TURRET_KD_CLOSE;
        dSwitch = TurretConstants.TURRET_D_SWITCH_DEGREES * tpd;
        dMin = TurretConstants.TURRET_D_MIN_DEGREES * tpd;
        dMax = TurretConstants.TURRET_D_MAX_DEGREES * tpd;
        kDFilter = TurretConstants.TURRET_D_FILTER;
        kVelocityFilter = TurretConstants.TURRET_VELOCITY_FILTER;
        kHold = TurretConstants.TURRET_K_HOLD_VOLTS;
        holdDecay = TurretConstants.TURRET_HOLD_DECAY_DEGREES * tpd;
        gain = TurretConstants.TURRET_GAIN;
        kStatic = TurretConstants.TURRET_BREAKAWAY_FRICTION;
        kCoulomb = TurretConstants.TURRET_KINETIC_FRICTION;
        stribeckVelocity = TurretConstants.TURRET_STRIBECK_DEGREES_PER_SECOND * tpd;
        frictionDeadband = TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES * tpd;
        frictionRelease = TurretConstants.TURRET_FRICTION_RELEASE_DEGREES * tpd;
        frictionRamp = TurretConstants.TURRET_FRICTION_RAMP_DEGREES_PER_SECOND * tpd;
        pulseEnabled = TurretConstants.TURRET_PULSE_ENABLED;
        pulseZone = TurretConstants.TURRET_PULSE_ZONE_DEGREES * tpd;
        pulseAimRate = TurretConstants.TURRET_PULSE_AIM_RATE_DEGREES_PER_SECOND * tpd;
        pulseOverBreakaway = TurretConstants.TURRET_PULSE_OVER_BREAKAWAY_VOLTS;
        frictionTrimStep = TurretConstants.TURRET_FRICTION_LEARN_STEP_VOLTS;
        frictionTrimMax = TurretConstants.TURRET_FRICTION_LEARN_MAX_VOLTS;
        pulseSeconds = TurretConstants.TURRET_PULSE_SECONDS;
        pulseMin = TurretConstants.TURRET_PULSE_MIN_SECONDS;
        pulseMax = TurretConstants.TURRET_PULSE_MAX_SECONDS;
        if (pulseLength != pulseLength) pulseLength = pulseSeconds;
        pulseWait = TurretConstants.TURRET_PULSE_WAIT_SECONDS;
        pulseStillVelocity = TurretConstants.TURRET_PULSE_STILL_DEGREES_PER_SECOND * tpd;
        pulseEnterVelocity = TurretConstants.TURRET_PULSE_ENTER_DEGREES_PER_SECOND * tpd;
        frictionFromVelocity = TurretConstants.TURRET_FRICTION_FROM_VELOCITY;
        stribeckFromMeasured = TurretConstants.TURRET_STRIBECK_FROM_MEASURED;
        double a = ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * tpd, b = ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * tpd;
        travelMin = Math.min(a, b);
        travelMax = Math.max(a, b);
        endDecel = TurretConstants.TURRET_END_DECEL_DEGREES_PER_S2 * tpd;
        endLag = TurretConstants.TURRET_END_LAG_SECONDS;
    }

    public void reset() {

        integral = 0;
        filteredDerivative = 0;
        positionalVelocity = 0;
        held = false;
        endPulses();
        first = true;
    }

    public double update(double dt, double target, double travelVelocity, double position) {

        double error = target - position;
        if (!first && dt > 0) measuredVelocity = (position - lastPosition) / dt;
        lastPosition = position;
        double magnitude = Math.abs(error);
        if (dt > 0) clock += dt;

        //small error, still aim: timed pushes shorter than the ~110 ms it takes to see motion, so a correction can't throw it far past
        //only once slowed down: coming in fast, the normal loop brakes it
        if (pulseEnabled && !first && magnitude <= pulseZone && Math.abs(travelVelocity) < pulseAimRate && (pulsing || Math.abs(measuredVelocity) < pulseEnterVelocity)) return pulse(error, magnitude, position);
        endPulses();

        p = (magnitude < pSwitch ? kpClose : kpFar) * error;

        double ki = magnitude > iZone ? 0 : magnitude < iCloseZone ? kiClose : kiFar;
        if (!first && dt > 0) integral += 0.5 * (prevError + error) * dt;
        if (Math.signum(error) != Math.signum(prevError)) integral *= kISmash;
        if (ki > 0) integral = clamp(integral, minI / ki, maxI / ki);
        i = ki * integral;

        double kd = magnitude < dSwitch ? kdClose : kdFar;
        double rawDerivative = !first && dt > 0 ? (error - prevError) / dt : 0;
        filteredDerivative = (1 - kDFilter) * filteredDerivative + kDFilter * rawDerivative;
        d = dt > 0 && magnitude >= dMin && magnitude <= dMax ? kd * filteredDerivative : 0;

        f = kHold * Math.signum(error) * (1 - Math.exp(-magnitude / holdDecay));

        positionalVelocity = (1 - kVelocityFilter) * positionalVelocity + kVelocityFilter * (p + i + d);
        double targetVelocity = positionalVelocity + travelVelocity;
        //never toward an end faster than it can still coast to a stop before it
        if (endDecel > 0 && targetVelocity != 0) {
            double room = (targetVelocity > 0 ? travelMax - position : position - travelMin) - Math.abs(measuredVelocity) * endLag;
            double cap = Math.sqrt(2 * endDecel * Math.max(0, room));
            if (Math.abs(targetVelocity) > cap) targetVelocity = Math.signum(targetVelocity) * cap;
        }
        v = gain > 0 ? targetVelocity / gain : 0;

        boolean still = Math.abs(travelVelocity) < stribeckVelocity;
        //once on target, no friction push until it's off by the release band: re-kicking every small drift is the shake
        held = still && (magnitude <= frictionDeadband || held && magnitude <= Math.max(frictionDeadband, frictionRelease));
        double sign = held ? 0 : Math.signum(frictionFromVelocity ? targetVelocity : error);
        double slip = stribeckFromMeasured ? measuredVelocity : targetVelocity;
        double stribeck = stribeckVelocity > 0 ? Math.exp(-(slip / stribeckVelocity) * (slip / stribeckVelocity)) : 0;
        //moving push shrinks with the asked speed, so a small correction can't run on friction push alone when friction is lower than thought
        double moving = frictionRamp > 0 ? Math.min(1, Math.abs(targetVelocity) / frictionRamp) : 1;
        double ks = kStatic + frictionTrim, kc = kStatic > 0 ? kCoulomb * ks / kStatic : kCoulomb;
        s = kc * sign * moving + (ks - kc * moving) * sign * stribeck;

        prevError = error;
        first = false;
        return v + s + f;
    }

    private double pulse(double error, double magnitude, double position) {

        //the normal loop's last push hasn't shown yet
        if (!pulsing) waitUntil = clock + pulseWait;
        if (pulseOn && clock >= pulseUntil) {
            pulseOn = false;
            waitUntil = clock + pulseWait;
        }
        boolean still = Math.abs(measuredVelocity) < pulseStillVelocity;
        if (!pulseOn && clock >= waitUntil && still) {
            if (pulsed) {
                //learn the push length from where the last one landed
                double moved = pulseSign * (position - pulseFrom);
                if (position == pulseFrom) pulsesWithoutCounts++;
                else pulsesWithoutCounts = 0;
                if (moved <= 0 && pulseLength >= pulseMax) frictionTrim = Math.min(frictionTrimMax, frictionTrim + frictionTrimStep);
                if (moved > pulseError + frictionDeadband && pulseLength <= pulseMin) frictionTrim = Math.max(-frictionTrimMax, frictionTrim - frictionTrimStep);
                if (moved <= 0) pulseLength *= 1.25;
                else if (moved > pulseError + frictionDeadband) pulseLength *= 0.75;
                else if (moved < 0.5 * pulseError) pulseLength *= 1.1;
                pulseLength = clamp(pulseLength, pulseMin, pulseMax);
                pulsed = false;
            }
            if (magnitude > frictionDeadband) {
                pulseOn = pulsed = true;
                pulseFrom = position;
                pulseSign = Math.signum(error);
                pulseError = magnitude;
                boolean outwardNearEnd = pulseSign > 0 ? travelMax - position < 2 * pulseZone : position - travelMin < 2 * pulseZone;
                pulseUntil = clock + (outwardNearEnd ? pulseMin : pulseLength);
            }
        }
        pulsing = true;
        integral = 0;
        positionalVelocity = 0;
        filteredDerivative = 0;
        p = i = d = f = v = 0;
        s = pulseOn ? pulseSign * (kStatic + frictionTrim + pulseOverBreakaway) : 0;
        prevError = error;
        first = false;
        return s;
    }

    private void endPulses() {

        pulsing = pulseOn = pulsed = false;
        pulsesWithoutCounts = 0;
        waitUntil = 0;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }
}
