package org.firstinspires.ftc.teamcode.Tuners.ShooterHyperTuning;

import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;

//tunes the turret while it's played: holding (aim still) and tracking (aim moving) are scored apart; holding while a shot is
//being fed counts double, that's when it has to be still
public class TurretHyperTuner {

    public final OnlineTuner hold, track;
    public double holdWindow = 2, trackWindow = 1.5, settle = 0.3, stillRate = 3, movingRate = 10, jump = 5, swing = 0.3, feedWeight = 2;
    public double lastHoldCost = Double.NaN, lastTrackCost = Double.NaN;

    private double holdTime, holdErr, holdSwings, holdFlips, holdSince = Double.NaN, ext = Double.NaN, dir, lastPos = Double.NaN, lastVolts;
    private double trackTime, trackErr, trackN, lastTarget = Double.NaN;

    public TurretHyperTuner() {

        hold = new OnlineTuner("turret hold", 2, new OnlineTuner.Knob("turret_kp_close", TurretConstants.TURRET_KP_CLOSE, 1, 20, 1, 0.2), new OnlineTuner.Knob("turret_kd_close", TurretConstants.TURRET_KD_CLOSE, 0, 1.5, 0.1, 0.02), new OnlineTuner.Knob("turret_friction_deadband_degrees", TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES, 0, 3, 0.25, 0.05), new OnlineTuner.Knob("turret_pulse_over_breakaway_volts", TurretConstants.TURRET_PULSE_OVER_BREAKAWAY_VOLTS, 0, 0.6, 0.05, 0.01), new OnlineTuner.Knob("turret_pulse_wait_seconds", TurretConstants.TURRET_PULSE_WAIT_SECONDS, 0.08, 0.4, 0.04, 0.01), new OnlineTuner.Knob("turret_pulse_zone_degrees", TurretConstants.TURRET_PULSE_ZONE_DEGREES, 0, 15, 2, 0.5));
        track = new OnlineTuner("turret track", 2, new OnlineTuner.Knob("turret_kp_far", TurretConstants.TURRET_KP_FAR, 0.5, 20, 1, 0.2), new OnlineTuner.Knob("turret_kd_far", TurretConstants.TURRET_KD_FAR, 0, 1.5, 0.1, 0.02), new OnlineTuner.Knob("turret_friction_ramp_degrees_per_second", TurretConstants.TURRET_FRICTION_RAMP_DEGREES_PER_SECOND, 0, 200, 20, 5), new OnlineTuner.Knob("turret_rate_feedforward_scale", TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE, 0.5, 1.3, 0.1, 0.02));
    }

    //tpd: ticks per degree, the controller works in ticks
    public void apply(TurretController c, double tpd) {

        c.kpClose = hold.value("turret_kp_close");
        c.kdClose = hold.value("turret_kd_close");
        c.frictionDeadband = hold.value("turret_friction_deadband_degrees") * tpd;
        c.pulseOverBreakaway = hold.value("turret_pulse_over_breakaway_volts");
        c.pulseWait = hold.value("turret_pulse_wait_seconds");
        c.pulseZone = hold.value("turret_pulse_zone_degrees") * tpd;
        c.kpFar = track.value("turret_kp_far");
        c.kdFar = track.value("turret_kd_far");
        c.frictionRamp = track.value("turret_friction_ramp_degrees_per_second") * tpd;
    }

    public double rateScale() {
        return track.value("turret_rate_feedforward_scale");
    }

    //degrees; aimRate is how fast the aim moves; returns true when a trial finished
    public boolean update(double now, double dt, double target, double position, double aimRate, double volts, boolean feeding) {

        boolean finished = false;
        boolean jumped = lastTarget == lastTarget && Math.abs(target - lastTarget) > jump;
        lastTarget = target;
        double error = Math.abs(target - position);

        if (!jumped && Math.abs(aimRate) < stillRate && dt > 0) {
            if (holdSince != holdSince) holdSince = now;
            if (now - holdSince > settle) {
                double w = feeding ? feedWeight : 1;
                holdErr += w * error * dt;
                holdTime += dt;
                if (lastVolts * volts < 0) holdFlips += w;
                if (lastPos == lastPos && position != lastPos) {
                    double d = Math.signum(position - lastPos);
                    if (d != dir) {
                        if (ext == ext && Math.abs(lastPos - ext) > swing) holdSwings += w;
                        ext = lastPos;
                        dir = d;
                    }
                }
                lastPos = position;
                if (holdTime >= holdWindow) {
                    lastHoldCost = (holdErr + 1.5 * holdSwings + 0.05 * holdFlips) / holdTime;
                    finished |= hold.submit(lastHoldCost);
                    holdTime = holdErr = holdSwings = holdFlips = 0;
                }
            }
        }
        else {
            holdSince = Double.NaN;
            lastPos = ext = Double.NaN;
            dir = 0;
        }

        if (!jumped && Math.abs(aimRate) >= movingRate && dt > 0) {
            trackErr += error;
            trackN++;
            trackTime += dt;
            if (trackTime >= trackWindow) {
                lastTrackCost = trackErr / trackN;
                finished |= track.submit(lastTrackCost);
                trackTime = trackErr = trackN = 0;
            }
        }
        lastVolts = volts;
        return finished;
    }
}
