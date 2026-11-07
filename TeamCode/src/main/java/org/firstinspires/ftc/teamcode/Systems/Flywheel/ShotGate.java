package org.firstinspires.ftc.teamcode.Systems.Flywheel;

public class ShotGate {

    private double enterBandBelow, enterBandAbove;
    private double holdBandBelow, holdBandAbove;
    private double dwell;
    private double minOpen;
    private double pulse;

    public ShotGate(double enterBandBelow, double enterBandAbove, double holdBandBelow, double holdBandAbove, double dwell, double minOpen, double pulse) {
        configure(enterBandBelow, enterBandAbove, holdBandBelow, holdBandAbove, dwell, minOpen, pulse);
    }

    public void configure(double enterBandBelow, double enterBandAbove, double holdBandBelow, double holdBandAbove, double dwell, double minOpen, double pulse) {

        this.enterBandBelow = enterBandBelow;
        this.enterBandAbove = enterBandAbove;

        this.holdBandBelow = Math.max(holdBandBelow, enterBandBelow);
        this.holdBandAbove = Math.max(holdBandAbove, enterBandAbove);

        this.dwell = dwell;
        this.minOpen = minOpen;
        this.pulse = pulse;
    }

    private boolean ready = false;
    private double inBandSince = Double.NaN;
    private double openedAt = Double.NaN;
    private boolean wasFiring = false;

    public boolean update(double now, double targetVelocity, double velocity, boolean firing) {

        if (firing && !wasFiring) openedAt = now;
        wasFiring = firing;

        if (!(targetVelocity > 0) || Double.isNaN(velocity) || Double.isInfinite(velocity)) {
            ready = false;
            inBandSince = Double.NaN;
            return false;
        }

        double error = velocity - targetVelocity;

        if (ready) {

            boolean outsideHoldBand = error < -holdBandBelow || error > holdBandAbove;
            boolean canClose = !firing || now - openedAt >= minOpen;

            boolean pulseDone = firing && pulse > 0 && now - openedAt >= pulse;

            if ((outsideHoldBand && canClose) || pulseDone) {
                ready = false;
                inBandSince = Double.NaN;
            }
        }
        else {

            boolean insideEnterBand = error >= -enterBandBelow && error <= enterBandAbove;

            if (!insideEnterBand) {
                inBandSince = Double.NaN;
            }
            else {
                if (Double.isNaN(inBandSince)) inBandSince = now;
                if (now - inBandSince >= dwell) ready = true;
            }
        }

        return ready;
    }

    public boolean isReady() {
        return ready;
    }

    public void reset() {

        ready = false;
        inBandSince = Double.NaN;
        openedAt = Double.NaN;
        wasFiring = false;
    }
}
