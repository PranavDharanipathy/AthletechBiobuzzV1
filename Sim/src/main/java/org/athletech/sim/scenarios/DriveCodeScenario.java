package org.athletech.sim.scenarios;

import org.athletech.sim.physics.Battery;
import org.athletech.sim.robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.DriveController;
import org.firstinspires.ftc.teamcode.Tuners.DriveAutotuneEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

final class DriveCodeScenario {

    private DriveCodeScenario() {}

    static final double IN = 39.3701;
    static final double H = 0.001;

    interface Sticks {

        void command(double t, double[] out);
    }

    interface Shove {

        void force(double t, double[] out);
    }

    static final class Trace {

        final List<double[]> rows = new ArrayList<>();

        double[] at(double t) {

            for (double[] r : rows) if (r[0] >= t) return r;
            return rows.get(rows.size() - 1);
        }
    }

    static final class Noise {

        double positionIn = 0.02, headingRad = Math.toRadians(0.1), velocityIn = 0.5, omegaRad = Math.toRadians(0.5);
        double commandDelay = 0.01;
        double jumpAt = Double.NaN, jumpX;

        Noise scaled(double scale, double delay) {

            Noise n = new Noise();
            n.positionIn = positionIn * scale;
            n.headingRad = headingRad * scale;
            n.velocityIn = velocityIn * scale;
            n.omegaRad = omegaRad * scale;
            n.commandDelay = delay;
            return n;
        }
    }

    static double maxDisplacement(Trace tr, double from, double to) {

        double[] start = tr.at(from);
        double m = 0;
        for (double[] r : tr.rows) if (r[0] >= from && r[0] <= to) m = Math.max(m, Math.hypot(r[1] - start[1], r[2] - start[2]));
        return m;
    }

    static double meanForward(Trace tr, double from, double to) {

        double sum = 0, n = 0;
        for (double[] r : tr.rows) {
            if (r[0] >= from && r[0] <= to) {
                sum += r[7];
                n++;
            }
        }
        return n > 0 ? sum / n : Double.NaN;
    }

    static Trace run(MecanumDrive.Params p, Battery battery, DriveController controller, Sticks sticks, Shove shove, double seconds, double loopSeconds, Noise noise, long seed) {

        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        Random rnd = new Random(seed);
        CodeLoop loop = new CodeLoop(loopSeconds, 0.3, seed + 3);
        Trace trace = new Trace();
        double[] cmd = new double[3], push = new double[3], wheels = new double[4], pending = new double[4], applied = new double[4];
        double t = 0, next = 0, last = 0, applyAt = Double.NaN;
        if (controller != null) controller.reset();
        while (t < seconds) {
            if (t >= next) {
                double dt = t - last;
                last = t;
                next += loop.next();
                sticks.command(t, cmd);
                double x = d.x() * IN + noise.positionIn * rnd.nextGaussian() + (t >= noise.jumpAt ? noise.jumpX : 0), y = d.y() * IN + noise.positionIn * rnd.nextGaussian();
                double h = d.heading() + noise.headingRad * rnd.nextGaussian();
                double vx = d.vx() * IN + noise.velocityIn * rnd.nextGaussian(), vy = d.vy() * IN + noise.velocityIn * rnd.nextGaussian();
                double w = d.omega() + noise.omegaRad * rnd.nextGaussian();
                if (controller != null) {
                    controller.update(dt, cmd, x, y, h, vx, vy, w, battery.lastVolts() > 1 ? battery.lastVolts() : 12.5);
                    DriveController.wheels(controller.power[0], controller.power[1], controller.power[2], wheels);
                }
                else DriveController.wheels(cmd[0], cmd[1], cmd[2], wheels);
                System.arraycopy(wheels, 0, pending, 0, 4);
                applyAt = t + noise.commandDelay;
                double c = Math.cos(d.heading()), s = Math.sin(d.heading());
                trace.rows.add(new double[] {t, d.x() * IN, d.y() * IN, d.heading(), d.vx() * IN, d.vy() * IN, d.omega(), (c * d.vx() + s * d.vy()) * IN, (-s * d.vx() + c * d.vy()) * IN});
            }
            if (t >= applyAt) {
                System.arraycopy(pending, 0, applied, 0, 4);
                applyAt = Double.NaN;
            }
            d.setPowers(applied[0], applied[1], applied[2], applied[3]);
            shove.force(t, push);
            d.push(push[0], push[1], push[2]);
            d.step(H, battery, 0);
            t += H;
        }
        return trace;
    }

    static final class Tune {

        DriveAutotuneEngine engine;
        double seconds, maxTravel;
    }

    static Tune autotune(MecanumDrive.Params p, Battery battery, double loopSeconds, Noise noise, long seed, double maxSeconds) {

        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        Random rnd = new Random(seed);
        CodeLoop loop = new CodeLoop(loopSeconds, 0.3, seed + 3);
        Tune r = new Tune();
        r.engine = new DriveAutotuneEngine();
        double[] wheels = new double[4], pending = new double[4], applied = new double[4];
        double t = 0, next = 0, applyAt = Double.NaN;
        while (t < maxSeconds && !r.engine.isDone() && !r.engine.isFaulted()) {
            if (t >= next) {
                next += loop.next();
                r.engine.update(t, d.x() * IN + noise.positionIn * rnd.nextGaussian(), d.y() * IN + noise.positionIn * rnd.nextGaussian(), d.heading() + noise.headingRad * rnd.nextGaussian(), d.vx() * IN + noise.velocityIn * rnd.nextGaussian(), d.vy() * IN + noise.velocityIn * rnd.nextGaussian(), d.omega() + noise.omegaRad * rnd.nextGaussian(), battery.lastVolts() > 1 ? battery.lastVolts() : 12.5);
                double[] u = r.engine.getPower();
                DriveController.wheels(u[0], u[1], u[2], wheels);
                System.arraycopy(wheels, 0, pending, 0, 4);
                applyAt = t + noise.commandDelay;
            }
            if (t >= applyAt) {
                System.arraycopy(pending, 0, applied, 0, 4);
                applyAt = Double.NaN;
            }
            d.setPowers(applied[0], applied[1], applied[2], applied[3]);
            d.step(H, battery, 0);
            r.maxTravel = Math.max(r.maxTravel, Math.hypot(d.x(), d.y()) * IN);
            t += H;
        }
        r.seconds = t;
        return r;
    }

    static DriveController matched(MecanumDrive.Params p, Battery battery, double loopSeconds) {

        DriveController c = new DriveController();
        DriveModel m = DriveModel.identify(p, battery, loopSeconds);
        for (int a = 0; a < 3; a++) {
            c.kS[a] = m.kS[a];
            c.kV[a] = m.kV[a];
            c.kA[a] = m.kA[a];
            c.maxAccel[a] = m.maxAccel[a];
            c.maxDecel[a] = m.maxDecel[a];
        }
        return c;
    }

    static double axisSpeed(MecanumDrive d, int a) {

        double c = Math.cos(d.heading()), s = Math.sin(d.heading());
        return a == 0 ? (c * d.vx() + s * d.vy()) * IN : a == 1 ? (-s * d.vx() + c * d.vy()) * IN : d.omega();
    }

    static final class DriveModel {

        final double[] kS = new double[3], kV = new double[3], kA = new double[3], maxAccel = new double[3], maxDecel = new double[3];

        static DriveModel identify(MecanumDrive.Params p, Battery battery, double loopSeconds) {

            DriveModel m = new DriveModel();
            for (int a = 0; a < 3; a++) {
                double[] powers = {0.4, 0.8};
                double[] speeds = new double[2], taus = new double[2];
                for (int k = 0; k < 2; k++) {
                    MecanumDrive d = new MecanumDrive(p);
                    d.reset(0, 0, 0);
                    double[] wheels = new double[4];
                    double[] u = new double[3];
                    u[a] = powers[k];
                    DriveController.wheels(u[0], u[1], u[2], wheels);
                    double t = 0, tau = Double.NaN;
                    List<double[]> speed = new ArrayList<>();
                    while (t < 3) {
                        d.setPowers(wheels[0], wheels[1], wheels[2], wheels[3]);
                        d.step(H, battery, 0);
                        t += H;
                        double c = Math.cos(d.heading()), s = Math.sin(d.heading());
                        double v = a == 0 ? (c * d.vx() + s * d.vy()) * IN : a == 1 ? (-s * d.vx() + c * d.vy()) * IN : d.omega();
                        speed.add(new double[] {t, v});
                    }
                    speeds[k] = speed.get(speed.size() - 1)[1];
                    for (double[] r : speed) {
                        if (r[1] >= 0.632 * speeds[k]) {
                            tau = r[0];
                            break;
                        }
                    }
                    taus[k] = tau;
                }
                m.kV[a] = (powers[1] - powers[0]) / (speeds[1] - speeds[0]);
                m.kS[a] = Math.max(0, powers[0] - m.kV[a] * speeds[0]);
                m.kA[a] = 0.5 * (taus[0] + taus[1]) * m.kV[a];
                MecanumDrive d = new MecanumDrive(p);
                d.reset(0, 0, 0);
                double[] wheels = new double[4], u = new double[3];
                double t = 0, lastV = 0, peak = 0, cruise = 0.7 * (1 - m.kS[a]) / m.kV[a], braking = Double.NaN, brakeFrom = 0;
                while (t < 4) {
                    double v = axisSpeed(d, a);
                    if (Double.isNaN(braking) && v >= cruise) {
                        braking = t;
                        brakeFrom = v;
                    }
                    u[a] = Double.isNaN(braking) ? 1 : -1;
                    DriveController.wheels(u[0], u[1], u[2], wheels);
                    d.setPowers(wheels[0], wheels[1], wheels[2], wheels[3]);
                    d.step(H, battery, 0);
                    t += H;
                    double now = axisSpeed(d, a);
                    if (Double.isNaN(braking)) peak = Math.max(peak, (now - lastV) / H);
                    lastV = now;
                    if (!Double.isNaN(braking) && now <= 0) {
                        m.maxDecel[a] = brakeFrom / (t - braking);
                        break;
                    }
                }
                m.maxAccel[a] = peak;
            }
            return m;
        }
    }
}
