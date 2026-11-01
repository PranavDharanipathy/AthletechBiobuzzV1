package org.firstinspires.ftc.teamcode.Testing.ShotExperiment;

/** Experiment-only positive-direction voltage feedforward + bounded PI; RPM in, duty out.
 * 12 V / 6000 RPM is a nominal motor estimate, not a measured loaded calibration.
 */
public final class ExperimentFlywheel {
    private double integralVolts, power;
    public void reset() { integralVolts=0; power=0; }
    public double update(double targetRpm, double measuredRpm, double batteryVolts, double dt) {
        if (!Double.isFinite(targetRpm) || targetRpm<=0 || targetRpm>5000
                || !Double.isFinite(measuredRpm) || !Double.isFinite(batteryVolts)
                || batteryVolts<10 || batteryVolts>14.5 || !Double.isFinite(dt) || dt<=0 || dt>.1) {
            reset(); return 0;
        }
        double error=targetRpm-measuredRpm;
        double feedforwardVolts=12*targetRpm/6000;
        double proportionalVolts=.001*error;
        double proposed=clamp(integralVolts+.0005*error*dt,-1.5,1.5);
        double requested=feedforwardVolts+proportionalVolts+proposed;
        double desiredPower=clamp(requested/batteryVolts,0,.9);
        double maxStep=.8*dt;
        double nextPower=clamp(desiredPower,Math.max(0,power-maxStep),Math.min(.9,power+maxStep));
        // Do not accumulate error against voltage saturation or the power slew limit.
        boolean constrainedHigh=requested>nextPower*batteryVolts+1e-9;
        boolean constrainedLow=requested<nextPower*batteryVolts-1e-9;
        if ((!constrainedHigh&&!constrainedLow) || (constrainedHigh&&error<0) || (constrainedLow&&error>0))
            integralVolts=proposed;
        power=nextPower; return power;
    }
    private static double clamp(double x,double low,double high) { return Math.max(low,Math.min(high,x)); }
}
