package org.athletech.sim.scenarios;

import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.junit.Test;

import java.util.Locale;

import static org.junit.Assert.assertTrue;

public class TurretControllerTest {

    static final double H = 0.0005, LOOP = 0.025;

    static double tpd() {
        return ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    static CrServoTurret plant(long seed) {
        return TurretPlants.measured(seed);
    }

    //{settle s (within 1 deg for good), overshoot deg, final error deg}
    static double[] step(double degrees, long seed) {

        CrServoTurret turret = plant(seed);
        TurretController c = new TurretController();
        c.applyConstants(tpd());
        double target = degrees * tpd(), t = 0, next = 0, last = 0, settled = Double.NaN, overshoot = 0;
        while (t < 4) {
            if (t >= next) {
                double volts = c.update(t - last, target, 0, turret.encoderPosition());
                last = t;
                next += LOOP;
                turret.command(t, volts, volts);
            }
            turret.step(t, H);
            double error = (target - turret.position()) / tpd();
            overshoot = Math.max(overshoot, -Math.signum(degrees) * error);
            if (Math.abs(error) > 1) settled = Double.NaN;
            else if (Double.isNaN(settled)) settled = t;
            t += H;
        }
        return new double[] {settled, overshoot, (target - turret.position()) / tpd()};
    }

    //share of loops within 2 deg of a field direction while the robot spins at spin rad/s
    static double spin(double spin, boolean cancel, long seed) {

        CrServoTurret turret = plant(seed);
        TurretController c = new TurretController();
        c.applyConstants(tpd());
        double t = 0, next = 0, last = 0, aimed = 0, n = 0;
        while (t < 6) {
            double omega = spin * Math.sin(Math.PI * t / 3);
            double heading = spin * 3 / Math.PI * (1 - Math.cos(Math.PI * t / 3));
            turret.setRobotAcceleration(-Math.toDegrees(spin * Math.PI / 3 * Math.cos(Math.PI * t / 3)) * tpd());
            double aim = -Math.toDegrees(heading) * tpd();
            if (t >= next) {
                double volts = c.update(t - last, aim, cancel ? -Math.toDegrees(omega) * tpd() : 0, turret.encoderPosition());
                last = t;
                next += LOOP;
                turret.command(t, volts, volts);
                if (t > 0.5 && Math.abs(aim - turret.position()) < 2 * tpd()) aimed++;
                if (t > 0.5) n++;
            }
            turret.step(t, H);
            t += H;
        }
        return aimed / n;
    }

    @Test
    public void stepsAndSpin() {

        for (double deg : new double[] {10, 30, 90}) {
            double[] r = step(deg, 1);
            System.out.printf(Locale.ROOT, "TURRET step %3.0f deg: settle %.2f s, overshoot %.1f deg, final %.2f deg%n", deg, r[0], r[1], r[2]);
            assertTrue("reaches " + deg, Math.abs(r[2]) < 2);
        }
        double with = spin(2, true, 2), without = spin(2, false, 2);
        System.out.printf(Locale.ROOT, "TURRET spin 2 rad/s: on aim %.0f%% with spin cancel, %.0f%% without%n", 100 * with, 100 * without);
        assertTrue("spin cancel helps", with > without);
    }

    //encoder reads 0 while the turret really moves (unplugged/uncoupled): TurretBase's cutoff must stop it well short of a turn
    //encoder unplugged from the start (cutoff waits TURRET_FROZEN_ENCODER_STARTUP_SECONDS) or mid-match after it counted (fast cutoff)
    static double[] deadEncoder(double deg, double unplugAt) {

        CrServoTurret turret = plant(4);
        if (unplugAt <= 0) turret.encoderSign = 0;
        TurretController c = new TurretController();
        c.applyConstants(tpd());
        TurretTuning.Limits limits = new TurretTuning.Limits();
        limits.controller = c;
        double t = 0, next = 0, last = 0, travel = 0, from = 0, target = unplugAt > 0 ? 20 * tpd() : deg * tpd(), cutAt = Double.NaN;
        while (t < unplugAt + 4) {
            if (unplugAt > 0 && t >= unplugAt && turret.encoderSign != 0) {
                turret.encoderSign = 0;
                from = turret.position();
                target = deg * tpd();
            }
            if (t >= next) {
                double position = turret.encoderPosition();
                double volts = limits.apply(t, c.update(t - last, target, 0, position), position);
                if (limits.frozen && Double.isNaN(cutAt)) cutAt = t;
                last = t;
                next += LOOP;
                double sent = t < TurretPlants.SERVO_WAKE ? 0 : volts;
                turret.command(t, sent, sent);
            }
            turret.step(t, H);
            if (t >= unplugAt) travel = Math.max(travel, Math.abs(turret.position() - from) / tpd());
            t += H;
        }
        return new double[] {limits.frozen ? 1 : 0, travel, cutAt - Math.max(0, unplugAt)};
    }

    @Test
    public void deadEncoderIsCaught() {

        for (double deg : new double[] {5, 30, 117, -150}) {
            double[] start = deadEncoder(deg, 0), mid = deadEncoder(deg, 3);
            System.out.printf(Locale.ROOT, "TURRET dead encoder, target %.0f deg: unplugged from the start cut off after %.1f s (ran %.0f deg); mid-match cut off after %.1f s (ran %.0f deg)%n", deg, start[2], start[1], mid[2], mid[1]);
            assertTrue("unplugged from the start caught", start[0] > 0 && start[2] < 3);
            assertTrue("unplugged from the start stopped inside the travel", start[1] < 180);
            assertTrue("unplugged mid-match caught", mid[0] > 0);
            assertTrue("mid-match stopped inside the travel", mid[1] < 180);
        }
    }
}
