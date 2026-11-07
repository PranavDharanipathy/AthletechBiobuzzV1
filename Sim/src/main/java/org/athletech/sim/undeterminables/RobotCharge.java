package org.athletech.sim.undeterminables;

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

    public void addFrame(double dq) {
        qf += dq;
    }

    public void step(double dt, double qInd, double gF, double gFE) {

        double a = dt * gF, b = dt * gFE;
        double d1 = pff - pfe, d2 = pee - pfe;
        double m11 = 1 + a * pff + b * d1, m12 = a * pfe - b * d2;
        double m21 = -b * d1, m22 = 1 + b * d2;
        double r1 = qf - (a * pff + b * d1) * qInd, r2 = qe + b * d1 * qInd;
        double det = m11 * m22 - m12 * m21;
        double nf = (r1 * m22 - m12 * r2) / det;
        double ne = (m11 * r2 - m21 * r1) / det;
        qf = nf;
        qe = ne;
    }

    public double equalizeInternal(double qInd) {

        double dv = vElec(qInd) - vFrame(qInd);
        double denom = pee + pff - 2 * pfe;
        double d = dv / denom;
        qe -= d;
        qf += d;
        return 0.5 * d * dv;
    }

    public double frameElastance() {
        return pff;
    }
}
