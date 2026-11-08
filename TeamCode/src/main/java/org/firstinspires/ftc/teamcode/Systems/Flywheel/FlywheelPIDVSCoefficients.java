package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSum;

public class FlywheelPIDVSCoefficients {

    public double kpFar, kpClose;
    public double kiFar, kiClose;
    public double kd;
    public double kv;
    public double ks;

    public double ksSignWidth;

    public double iSwitch;
    public double pSwitch;

    public double kISmash;

    public double minP, maxP;
    public double minI, maxI;
    public double minD, maxD;

    public FlywheelPIDVSCoefficients(
            double kpFar,
            double kpClose,
            double kiFar,
            double kiClose,
            double kd,
            double kv,
            double ks,
            double ksSignWidth,
            double iSwitch,
            double pSwitch,
            double kISmash,
            double minP,
            double maxP,
            double minI,
            double maxI,
            double minD,
            double maxD
    ) {

        updateCoefficients(kpFar, kpClose, kiFar, kiClose, kd, kv, ks, ksSignWidth, iSwitch, pSwitch, kISmash, minP, maxP, minI, maxI, minD, maxD);
    }

    public void updateCoefficients(
            double kpFar,
            double kpClose,
            double kiFar,
            double kiClose,
            double kd,
            double kv,
            double ks,
            double ksSignWidth,
            double iSwitch,
            double pSwitch,
            double kISmash,
            double minP,
            double maxP,
            double minI,
            double maxI,
            double minD,
            double maxD
    ) {

        this.kpFar = kpFar;
        this.kpClose = kpClose;

        this.kiFar = kiFar;
        this.kiClose = kiClose;

        this.kd = kd;

        this.kv = kv;

        this.ks = ks;
        this.ksSignWidth = ksSignWidth;

        this.iSwitch = iSwitch;
        this.pSwitch = pSwitch;

        this.kISmash = kISmash;

        this.minP = minP;
        this.maxP = maxP;

        this.minI = minI;
        this.maxI = maxI;

        this.minD = minD;
        this.maxD = maxD;
    }

    public double kp(double targetVelocity, double currentVelocity) {

        if (Math.abs(targetVelocity - currentVelocity) < pSwitch) return kpClose;
        else return kpFar;
    }

    private double kISwitchTargetVelocity;

    public double ki(double targetVelocity, double currentVelocity, DynamicTrapezoidalSum errorSum) {

        if (kISwitchTargetVelocity == targetVelocity || Math.abs(targetVelocity - currentVelocity) < iSwitch) {

            if (kISwitchTargetVelocity != targetVelocity) errorSum.setSum(0);

            kISwitchTargetVelocity = targetVelocity;
            return kiClose;
        }
        else return kiFar;
    }
}
