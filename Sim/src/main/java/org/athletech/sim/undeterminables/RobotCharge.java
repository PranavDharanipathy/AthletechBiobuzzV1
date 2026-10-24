package org.athletech.sim.undeterminables;

/**
 * Static charge on one robot as two conductors: the metal frame and the electronics (battery
 * negative, hubs, wiring). Their Maxwell capacitance matrix is
 * <pre>
 *   [ Cf + Cfe   -Cfe     ]      Cf  = frame to floor
 *   [ -Cfe       Ce + Cfe ]      Ce  = electronics to floor, Cfe = electronics to frame
 * </pre>
 * and potentials are {@code V = P (Q + induced)} with {@code P} its inverse (elastance). Charge
 * tracked: {@code qf} on the frame, {@code qe} on the electronics. Tile charge under the robot
 * induces an extra effective charge on the frame ({@code qInd}).
 *
 * <p>Leakage is integrated implicitly (backward Euler), so very small time constants (a 470 ohm
 * grounding strap equalizes in microseconds) stay stable at any step: the frame leaks to the
 * floor through {@code gF} (wheels, tiles, corona) and to the electronics through {@code gFE}
 * (strap or insulation).
 */
public final class RobotCharge {
    public final double cFrame, cElec, cCouple;
    public final double pff, pfe, pee;
    private double qf, qe;

    public RobotCharge(double cFrame, double cElec, double cCouple) {
        this.cFrame = cFrame;
        this.cElec = cElec;
        this.cCouple = cCouple;
        double c11 = cFrame + cCouple, c22 = cElec + cCouple, c12 = -cCouple;
        double det = c11 * c22 - c12 * c12;
        this.pff = c22 / det;
        this.pee = c11 / det;
        this.pfe = -c12 / det;
    }

    public double qFrame() { return qf; }

    public double qElec() { return qe; }

    public void reset() {
        qf = 0;
        qe = 0;
    }

    public double vFrame(double qInd) {
        return pff * (qf + qInd) + pfe * qe;
    }

    public double vElec(double qInd) {
        return pfe * (qf + qInd) + pee * qe;
    }

    /** Adds charge to the frame (friction, contact charging, a discharge). */
    public void addFrame(double dq) {
        qf += dq;
    }

    /**
     * Advances {@code dt} seconds: frame-to-floor conductance {@code gF}, frame-to-electronics
     * conductance {@code gFE} (both S), with {@code qInd} held constant over the step.
     */
    public void step(double dt, double qInd, double gF, double gFE) {
        double a = dt * gF, b = dt * gFE;
        double d1 = pff - pfe, d2 = pee - pfe;
        // row 1: qf'(1 + a pff + b d1) + qe'(a pfe - b d2) = qf - (a pff + b d1) qInd
        // row 2: qf'(-b d1)            + qe'(1 + b d2)     = qe + b d1 qInd
        double m11 = 1 + a * pff + b * d1, m12 = a * pfe - b * d2;
        double m21 = -b * d1, m22 = 1 + b * d2;
        double r1 = qf - (a * pff + b * d1) * qInd, r2 = qe + b * d1 * qInd;
        double det = m11 * m22 - m12 * m21;
        double nf = (r1 * m22 - m12 * r2) / det;
        double ne = (m11 * r2 - m21 * r1) / det;
        qf = nf;
        qe = ne;
    }

    /**
     * Arc between electronics and frame: they reach the same potential. Returns the energy
     * released (J), all of which goes through the electronics.
     */
    public double equalizeInternal(double qInd) {
        double dv = vElec(qInd) - vFrame(qInd);
        double denom = pee + pff - 2 * pfe;
        double d = dv / denom;
        qe -= d;
        qf += d;
        return 0.5 * d * dv;
    }

    /** Elastance seen by charge moving onto/off the frame (1/F), for contact equalization. */
    public double frameElastance() {
        return pff;
    }
}
