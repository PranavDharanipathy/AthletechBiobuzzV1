package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

public final class AutoShotWindow {

    public enum State { IDLE, WAITING, FIRING, DONE, TIMED_OUT, FAULT }
    private State state = State.IDLE;
    private double start, last, inBandSince, fed;
    private double feedSeconds, timeout, dwell, maxLoop;

    public void start(double now, double feedSeconds, double timeout, double dwell, double maxLoop) {

        if (!Double.isFinite(now) || !positive(feedSeconds) || !positive(timeout) || !positive(dwell) || !positive(maxLoop) || feedSeconds >= timeout || maxLoop > 0.2) throw new IllegalArgumentException("Invalid auto shot timing");
        this.feedSeconds = feedSeconds;
        this.timeout = timeout;
        this.dwell = dwell;
        this.maxLoop = maxLoop;
        start = last = now;
        fed = 0;
        inBandSince = Double.NaN;
        state = State.WAITING;
    }

    public void update(double now, boolean ready) {

        if (finished() || state == State.IDLE) return;
        double dt = now - last;
        last = now;
        if (!Double.isFinite(now) || dt < 0 || dt > maxLoop) { state = State.FAULT; return; }
        if (state == State.FIRING) fed += dt;
        if (now - start >= timeout) { state = State.TIMED_OUT; return; }
        if (fed >= feedSeconds) { state = State.DONE; return; }
        if (!ready) {
            inBandSince = Double.NaN;
            state = State.WAITING;
            return;
        }
        if (Double.isNaN(inBandSince)) inBandSince = now;
        state = now - inBandSince >= dwell ? State.FIRING : State.WAITING;
    }

    public boolean firing() { return state == State.FIRING; }
    public boolean finished() { return state == State.DONE || state == State.TIMED_OUT || state == State.FAULT; }
    public State state() { return state; }
    public double fedSeconds() { return fed; }
    private static boolean positive(double value) { return Double.isFinite(value) && value > 0; }
}
