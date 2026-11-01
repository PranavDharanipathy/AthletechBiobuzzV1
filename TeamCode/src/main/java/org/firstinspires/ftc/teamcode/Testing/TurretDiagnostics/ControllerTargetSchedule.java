package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

/**
 * Target sequence for the existing-controller test, in degrees from the starting reference.
 *
 * <p>Every run first holds the position where it started for one dwell, so the baseline
 * controller begins with near-zero error. That avoids a startup step that would otherwise be
 * mistaken for controller behavior. After that it visits each target for one dwell and completes.
 * With {@code holdForever}, it moves to the first target and stays there until the operator stops.
 * That plan is for watching the hold while the turret or robot is disturbed by hand.
 */
public final class ControllerTargetSchedule {

    private final double[] targetsDeg;
    private final double dwellS;
    private final boolean holdForever;

    private long startNanos;
    private double startDeg;
    private boolean running;
    private boolean complete;
    private int phase = -1;

    public ControllerTargetSchedule(double[] targetsDeg, double dwellS, boolean holdForever) {
        if (targetsDeg.length == 0) throw new IllegalArgumentException("no targets");
        this.targetsDeg = targetsDeg.clone();
        this.dwellS = dwellS;
        this.holdForever = holdForever;
    }

    public void start(long nanos, double currentDeg) {
        startNanos = nanos;
        startDeg = currentDeg;
        running = true;
        complete = false;
        phase = -1;
    }

    /** @return the target for this loop in degrees, or NaN once complete. */
    public double update(long nanos) {
        if (!running) return Double.NaN;
        double elapsed = (nanos - startNanos) * 1e-9;
        int p = (int) Math.floor(elapsed / dwellS) - 1;
        if (holdForever) {
            phase = Math.min(p, 0);
            return phase < 0 ? startDeg : targetsDeg[0];
        }
        if (p >= targetsDeg.length) {
            running = false;
            complete = true;
            return Double.NaN;
        }
        phase = p;
        return p < 0 ? startDeg : targetsDeg[p];
    }

    public void stop() { running = false; }
    public boolean isComplete() { return complete; }
    /** -1 while holding the start position, then the index of the current target. */
    public int phase() { return phase; }
}
