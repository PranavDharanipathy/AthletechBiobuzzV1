package org.firstinspires.ftc.teamcode.Systems.Flywheel;

//holds a ball back when the wheel won't be inside this shot's window by the time that ball reaches it. Balls closer than `commit`
//to the wheel are already past the blocker: they'll hit no matter what, so the prediction takes their speed off first. Open: it
//judges the first ball it can still stop; shut: one let go now, firstContact later
public final class FeedGate {

    public interface Predictor {
        //the wheel's speed `seconds` from now, with a ball hitting at each of the first `balls` of `hits` (seconds from now)
        double speedIn(double seconds, double[] hits, int balls);
    }

    //early: balls come up to this much sooner than expected, judge them then (a recovering wheel is slower earlier); margins keep
    //clear of the window's edges, more to let one go than to keep going
    public double commit = 0.12, early = 0.05, stayMargin = 8, enterMargin = 15, minClosed = 0.06;
    //a trigger press at least this long that got no ball out (the blocker never stayed open long enough to send one)
    public double starveSeconds = 0.4;
    //the first ball of an opening only counts as past the blocker this long after `commit` says so: shutting for the next ball
    //while the first one isn't really through stops both, and a gate that does that every time never lets a ball out
    public double firstGuard = 0.04;
    public double predictedError = Double.NaN, judgedIn = Double.NaN;
    public boolean open;
    public int holds, committed, starved;
    public double heldSeconds;
    private double closedAt = Double.NEGATIVE_INFINITY, last = Double.NaN, pressStart = Double.NaN, openedAt = Double.NaN;
    private boolean wasWanted, pressFed;
    private final double[] hits = new double[8], pending = new double[8];
    private int pendingCount;

    //nextContact: when the next ball is due while open (NaN = unknown)
    public boolean update(double now, boolean wanted, double target, Predictor predictor, double nextContact, double firstContact, double interval, double below, double above) {

        double dt = Double.isNaN(last) ? 0 : now - last;
        last = now;
        if (wanted && !wasWanted) {
            pressStart = now;
            pressFed = false;
        }
        if (!wanted && wasWanted && !pressFed && now - pressStart >= starveSeconds) starved++;
        wasWanted = wanted;
        int balls = 0;
        double contact = now + firstContact;
        boolean through = open && now - openedAt >= firstContact - commit + firstGuard;
        if (open) {
            contact = Double.isNaN(nextContact) || nextContact < now - 0.1 ? now + interval : nextContact;
            if (!through) contact = Math.max(now, openedAt + firstContact);
            while (through && contact - now < commit && balls < hits.length) {
                hits[balls++] = Math.max(0, contact - now);
                contact += Math.max(0.05, interval);
            }
        }
        else {
            for (int k = 0; k < pendingCount; k++) if (pending[k] > now - 0.05 && balls < hits.length) hits[balls++] = Math.max(0, pending[k] - now);
        }
        committed = balls;
        if (!wanted || !(target > 0)) {
            close(now, balls);
            predictedError = Double.NaN;
            return false;
        }
        judgedIn = Math.max(0, contact - now - early);
        predictedError = predictor.speedIn(judgedIn, hits, balls) - target;
        boolean stay = predictedError >= -(below - stayMargin) && predictedError <= above - stayMargin;
        boolean enter = predictedError >= -(below - enterMargin) && predictedError <= above - enterMargin;
        if (open && !stay) {
            close(now, balls);
            holds++;
        }
        else if (!open && enter && now - closedAt >= minClosed) {
            open = true;
            openedAt = now;
            pendingCount = 0;
        }
        if (!open) heldSeconds += dt;
        else if (through) pressFed = true;
        return open;
    }

    //the balls already through still land
    private void close(double now, int balls) {

        if (open) {
            closedAt = now;
            pendingCount = 0;
            for (int k = 0; k < balls; k++) pending[pendingCount++] = now + hits[k];
        }
        open = false;
    }

    public void reset() {

        open = false;
        predictedError = judgedIn = Double.NaN;
        closedAt = Double.NEGATIVE_INFINITY;
        last = pressStart = openedAt = Double.NaN;
        wasWanted = false;
        pendingCount = committed = 0;
    }
}
