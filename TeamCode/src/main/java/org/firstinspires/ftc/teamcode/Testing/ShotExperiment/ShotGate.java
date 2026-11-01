package org.firstinspires.ftc.teamcode.Testing.ShotExperiment;

/** Stationary experiment sequencer. Seconds, RPM, RPM/s. No queued shots or hardware dependencies. */
public final class ShotGate {
    public enum State { BLOCKED, SETTLING, READY, FEEDING, RECOVERING }
    public static final class Settings {
        public final double rpmTolerance, accelerationLimit, stableSeconds, maxLoopSeconds;
        public final double minVoltage, feedSeconds, recoverySeconds;
        public Settings(double tolerance, double acceleration, double stable, double maxLoop,
                        double minVoltage, double feed, double recovery) {
            if (!positive(tolerance) || !positive(acceleration) || !positive(stable)
                    || !positive(maxLoop) || !positive(minVoltage)
                    || !positive(feed) || !positive(recovery) || stable < maxLoop
                    || recovery < maxLoop || feed < maxLoop || maxLoop > .1 || feed > .5
                    || stable > 5 || recovery > 10 || tolerance > 250 || acceleration > 2000
                    || minVoltage < 10 || minVoltage > 14)
                throw new IllegalArgumentException("Invalid experiment timing/readiness settings");
            rpmTolerance=tolerance; accelerationLimit=acceleration; stableSeconds=stable;
            maxLoopSeconds=maxLoop; this.minVoltage=minVoltage;
            feedSeconds=feed; recoverySeconds=recovery;
        }
    }
    private final Settings settings;
    private State state=State.BLOCKED;
    private double lastTime=Double.NaN, lastRpm, lastTarget=Double.NaN;
    private double stableSince=Double.NaN, stageSince, recoverUntil;
    private boolean lastTrigger;
    private String reason="Not armed";
    private int attempts;
    public ShotGate(Settings settings) { this.settings=settings; }
    private static boolean positive(double x) { return Double.isFinite(x) && x>0; }
    public State state() { return state; }
    public String reason() { return reason; }
    public int attempts() { return attempts; }
    public boolean feed() { return state==State.FEEDING; }

    public void update(double now, double target, double rpm, double volts,
                       boolean armed, boolean geometryFixed, boolean stationary, boolean trigger) {
        boolean edge=trigger && !lastTrigger;
        lastTrigger=trigger; // Even blocked edges are consumed: never fire later from a stale press.
        double dt=now-lastTime;
        double acceleration=(rpm-lastRpm)/dt;
        boolean sample=Double.isFinite(now) && Double.isFinite(rpm) && Double.isFinite(volts)
                && volts<=14.5 && positive(target) && Double.isFinite(dt) && dt>0 && dt<=settings.maxLoopSeconds;
        boolean targetChanged=target!=lastTarget;
        lastTime=Double.isFinite(now) ? now : Double.NaN; lastRpm=rpm; lastTarget=target;
        if (!sample || targetChanged || !armed || !geometryFixed || !stationary || volts<settings.minVoltage) {
            state=State.BLOCKED; stableSince=Double.NaN;
            reason=!sample ? "Invalid sample / loop gap" : targetChanged ? "Target changed"
                    : !armed ? "Hold RB to arm" : !geometryFixed ? "Secure fixed geometry"
                    : !stationary ? "Robot/turret moving" : "Low voltage";
            return;
        }
        // RPM dip during feeding is expected. Global interlocks above still abort immediately.
        if (state==State.FEEDING) {
            if (now-stageSince>=settings.feedSeconds) {
                state=State.RECOVERING; recoverUntil=now+settings.recoverySeconds;
                stableSince=Double.NaN; reason="Flywheel recovery";
            }
            return;
        }
        boolean stable=Math.abs(target-rpm)<=settings.rpmTolerance
                && Double.isFinite(acceleration) && Math.abs(acceleration)<=settings.accelerationLimit;
        if (!stable) { stableSince=Double.NaN; state=State.SETTLING; reason="RPM error / acceleration"; return; }
        if (!Double.isFinite(stableSince)) stableSince=now;
        if (now<recoverUntil) { state=State.RECOVERING; reason="Minimum shot spacing"; return; }
        if (now-stableSince<settings.stableSeconds) { state=State.SETTLING; reason="Continuous stability dwell"; return; }
        state=State.READY; reason="Ready: fresh A press";
        if (edge) {
            state=State.FEEDING; stageSince=now; attempts++;
            recoverUntil=now+settings.feedSeconds+settings.recoverySeconds;
            reason="Timed intake feed; physical release unmeasured";
        }
    }
}
