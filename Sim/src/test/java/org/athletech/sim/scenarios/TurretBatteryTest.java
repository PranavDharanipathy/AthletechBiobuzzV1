package org.athletech.sim.scenarios;

import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.junit.Test;

import java.util.Locale;
import java.util.Random;

import static org.junit.Assert.assertTrue;

//a match's battery drain (12.6 -> 10.5 V, dips while driving) with the turret holding a hopping aim: servo supply regulated or on
//the battery, friction as tuned or 15% off; the hold must stay on target with no lurches the whole way
public class TurretBatteryTest {

    static final double H = 0.0005, SECONDS = 150;

    //per third of the match {mean |err| deg, biggest swing deg}
    static double[][] run(boolean servoOnBattery, double frictionScale, long seed) {

        double tpd = TurretControllerTest.tpd();
        Random r = new Random(seed);
        CrServoTurret turret = new CrServoTurret(TurretPlants.params(1, frictionScale, frictionScale), seed);
        TurretController c = new TurretController();
        c.applyConstants(tpd);
        double base = 0, t = 0, next = 0, last = 0, hopAt = 1, dipUntil = 0, lastPos = Double.NaN, dir = 0, ext = Double.NaN, hopped = -10;
        double[][] out = new double[3][3];
        while (t < SECONDS) {
            double battery = 12.6 - 2.1 * t / SECONDS - (t < dipUntil ? 0.8 : 0);
            if (t >= next) {
                if (t > hopAt) {
                    base += (r.nextBoolean() ? 1 : -1) * (0.5 + 10 * r.nextDouble() * r.nextDouble()) * tpd;
                    hopAt += 1 + 3 * r.nextDouble();
                    hopped = t;
                    if (r.nextDouble() < 0.3) dipUntil = t + 0.5 + 1.5 * r.nextDouble();
                }
                double pos = turret.encoderPosition();
                double cmd = c.update(t - last, base + 0.1 * tpd * r.nextGaussian(), 0, pos);
                last = t;
                next += 0.03 * (0.8 + 0.4 * r.nextDouble());
                double supply = TurretConstants.TURRET_VOLTAGE_COMPENSATION ? battery : TurretConstants.TURRET_NOMINAL_VOLTS;
                double power = Math.max(-1, Math.min(1, cmd / supply));
                double servoVolts = power * (servoOnBattery ? battery : TurretConstants.TURRET_NOMINAL_VOLTS);
                turret.command(t, servoVolts, servoVolts);
                int third = (int) Math.min(2, t / (SECONDS / 3));
                //judge the hold once a hop has had time to land
                if (t > 2 && t - hopped > 0.8) {
                    out[third][0] += Math.abs(base - pos) / tpd;
                    out[third][2]++;
                    if (!Double.isNaN(lastPos) && pos != lastPos) {
                        double d = Math.signum(pos - lastPos);
                        if (d != dir) {
                            if (!Double.isNaN(ext)) out[third][1] = Math.max(out[third][1], Math.abs(lastPos - ext) / tpd);
                            ext = lastPos;
                            dir = d;
                        }
                    }
                    lastPos = pos;
                }
                else {
                    ext = lastPos = Double.NaN;
                    dir = 0;
                }
            }
            turret.step(t, H);
            t += H;
        }
        for (double[] o : out) o[0] /= Math.max(1, o[2]);
        return out;
    }

    @Test
    public void holdsThroughTheDrain() {

        for (boolean onBattery : new boolean[] {false, true}) {
            for (double fs : new double[] {1, 0.85, 1.15}) {
                for (long seed = 1; seed <= 2; seed++) {
                    double[][] x = run(onBattery, fs, seed);
                    System.out.printf(Locale.ROOT, "TURRET drain servos %s friction x%.2f seed %d: err %.2f / %.2f / %.2f deg, biggest swing %.1f / %.1f / %.1f deg%n", onBattery ? "battery" : "regulated", fs, seed, x[0][0], x[1][0], x[2][0], x[0][1], x[1][1], x[2][1]);
                    for (int k = 0; k < 3; k++) {
                        assertTrue("on target, third " + k, x[k][0] < 2);
                        assertTrue("no lurch, third " + k, x[k][1] < 12);
                    }
                }
            }
        }
    }
}
