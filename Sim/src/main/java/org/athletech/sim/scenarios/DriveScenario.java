package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.predict.DriveTimePredictor.Axis;
import org.athletech.sim.predict.DriveTimePredictor;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.telemetry.Recorder;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * Drivetrain sims: full-power runs (forward, strafe, turn, braking), stop-to-stop move times, and
 * how all of it changes as the battery drains. Fills the drive.* derived entries.
 */
public final class DriveScenario {
    private DriveScenario() {
    }

    static final double DT = 0.001;
    static final double[] DISTANCES = {0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.5};
    static final double[] ANGLES = {15, 30, 45, 60, 90, 120, 150, 180, 270, 360};
    static final double[] SURFACE_DISTANCES = {0.3, 0.6, 1.0, 1.5, 2.0, 3.0};

    /** Results of a full-power run from rest. */
    static final class Run {
        private final double topSpeed;
        private final double timeTo95;
        private final double peakAccel;
        private final double peakCurrent;
        private final double cruiseCurrent;
        private final Recorder rec;

        Run(double topSpeed, double timeTo95, double peakAccel, double peakCurrent, double cruiseCurrent, Recorder rec) {
            this.topSpeed = topSpeed;
            this.timeTo95 = timeTo95;
            this.peakAccel = peakAccel;
            this.peakCurrent = peakCurrent;
            this.cruiseCurrent = cruiseCurrent;
            this.rec = rec;
        }

        public double topSpeed() { return topSpeed; }
        public double timeTo95() { return timeTo95; }
        public double peakAccel() { return peakAccel; }
        public double peakCurrent() { return peakCurrent; }
        public double cruiseCurrent() { return cruiseCurrent; }
        public Recorder rec() { return rec; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Run)) return false;
            Run o = (Run) other;
            return Double.compare(topSpeed, o.topSpeed) == 0
                    && Double.compare(timeTo95, o.timeTo95) == 0
                    && Double.compare(peakAccel, o.peakAccel) == 0
                    && Double.compare(peakCurrent, o.peakCurrent) == 0
                    && Double.compare(cruiseCurrent, o.cruiseCurrent) == 0
                    && java.util.Objects.equals(rec, o.rec);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(topSpeed, timeTo95, peakAccel, peakCurrent, cruiseCurrent, rec);
        }

        @Override
        public String toString() {
            return "Run[topSpeed=" + topSpeed + ", timeTo95=" + timeTo95 + ", peakAccel=" + peakAccel + ", peakCurrent=" + peakCurrent + ", cruiseCurrent=" + cruiseCurrent + ", rec=" + rec + "]";
        }
    }

    static Run fullPower(MecanumDrive.Params p, Battery battery, double otherA, Axis axis, double seconds) {
        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        Recorder rec = new Recorder(0.01, "t_s", "speed", "motor_current_a", "battery_current_a", "bus_v");
        double t = 0, prev = 0, peakA = 0, peakI = 0;
        while (t < seconds - 1e-9) {
            switch (axis) {
                case FORWARD:
                    d.drive(1, 0, 0);
                    break;
                case STRAFE:
                    d.drive(0, 1, 0);
                    break;
                case TURN:
                    d.drive(0, 0, 1);
                    break;
            }
            d.step(DT, battery, otherA);
            t += DT;
            double v = Math.abs(DriveTimePredictor.velocityOf(d, axis));
            peakA = Math.max(peakA, (v - prev) / DT);
            prev = v;
            peakI = Math.max(peakI, d.motorCurrentTotal());
            rec.sample(t, v, d.motorCurrentTotal(), d.batteryCurrent(), d.busVolts());
        }
        double[] sp = rec.column("speed");
        double top = sp[sp.length - 1];
        double t95 = Double.NaN;
        double[] ts = rec.column("t_s");
        for (int i = 0; i < sp.length; i++) {
            if (sp[i] >= 0.95 * top) {
                t95 = ts[i];
                break;
            }
        }
        return new Run(top, t95, peakA, peakI, d.motorCurrentTotal(), rec);
    }

    /** Peak deceleration reversing power at top speed. */
    static double peakDecel(MecanumDrive.Params p, Battery battery, double otherA) {
        MecanumDrive d = new MecanumDrive(p);
        d.reset(0, 0, 0);
        for (double t = 0; t < 3; t += DT) {
            d.drive(1, 0, 0);
            d.step(DT, battery, otherA);
        }
        double prev = d.forwardSpeed(), peak = 0;
        for (double t = 0; t < 0.5 && d.forwardSpeed() > 0; t += DT) {
            d.drive(-1, 0, 0);
            d.step(DT, battery, otherA);
            peak = Math.max(peak, (prev - d.forwardSpeed()) / DT);
            prev = d.forwardSpeed();
        }
        return peak;
    }

    public static void run(SimContext ctx) throws Exception {
        RobotParams rp = new RobotParams(ctx.db);
        MecanumDrive.Params p = rp.drive();
        rp.battery(Double.NaN);
        double other = rp.electronicsCurrent();
        rp.requireAll();
        double voc = ctx.db.value("battery.open_circuit_v");
        Supplier<Battery> fresh = () -> rp.battery(voc);
        SimContext.Saver save = ctx.saver(rp, "sim drive");
        ctx.log.println("Drive sim  (mass " + p.massKg() + " kg, theoretical free speed " + Reports.round(p.theoreticalTopSpeed()) + " m/s, battery " + voc + " V)");

        Run fwd = fullPower(p, fresh.get(), other, Axis.FORWARD, 3);
        Run strafe = fullPower(p, fresh.get(), other, Axis.STRAFE, 3);
        Run turn = fullPower(p, fresh.get(), other, Axis.TURN, 3);
        double decel = peakDecel(p, fresh.get(), other);
        double turnDps = Math.toDegrees(turn.topSpeed());
        ctx.log.printf("  top speed %.2f m/s forward, %.2f m/s strafe, %.0f deg/s turning%n", fwd.topSpeed(), strafe.topSpeed(), turnDps);
        ctx.log.printf("  peak accel %.2f m/s^2, peak decel %.2f m/s^2, 95%% speed after %.2f s, peak current %.1f A%n",
                fwd.peakAccel(), decel, fwd.timeTo95(), fwd.peakCurrent());

        List<double[]> speedVsTime = new ArrayList<>(), currentVsTime = new ArrayList<>(), currentVsSpeed = new ArrayList<>(), busVsTime = new ArrayList<>();
        double[] ts = fwd.rec().column("t_s"), sp = fwd.rec().column("speed"), cur = fwd.rec().column("motor_current_a"), bus = fwd.rec().column("bus_v");
        for (int i = 0; i < ts.length; i += 5) {
            speedVsTime.add(new double[]{ts[i], sp[i]});
            currentVsTime.add(new double[]{ts[i], cur[i]});
            busVsTime.add(new double[]{ts[i], bus[i]});
        }
        double lastBin = -1;
        for (int i = 0; i < sp.length; i++) {
            double bin = Math.floor(sp[i] / (fwd.topSpeed() / 20));
            if (bin > lastBin) {
                currentVsSpeed.add(new double[]{sp[i], cur[i]});
                lastBin = bin;
            }
        }

        List<double[]> tFwd = new ArrayList<>(), tStrafe = new ArrayList<>(), tTurn = new ArrayList<>();
        for (double dist : DISTANCES) {
            tFwd.add(new double[]{dist, DriveTimePredictor.timeTo(p, fresh, other, Axis.FORWARD, dist).timeS()});
            tStrafe.add(new double[]{dist, DriveTimePredictor.timeTo(p, fresh, other, Axis.STRAFE, dist).timeS()});
        }
        for (double a : ANGLES) tTurn.add(new double[]{a, DriveTimePredictor.timeTo(p, fresh, other, Axis.TURN, Math.toRadians(a)).timeS()});
        ctx.log.printf("  stop-to-stop: 1 m in %.2f s, 2 m in %.2f s, 90 deg turn in %.2f s%n", interp(tFwd, 1), interp(tFwd, 2), interp(tTurn, 90));

        List<double[]> topVsV = new ArrayList<>(), accelVsV = new ArrayList<>(), surface = new ArrayList<>();
        for (double v = 11.5; v <= 13.75; v += 0.25) {
            final double vv = v;
            Run r = fullPower(p, rp.battery(v), other, Axis.FORWARD, 2.5);
            topVsV.add(new double[]{v, r.topSpeed()});
            accelVsV.add(new double[]{v, r.peakAccel()});
            if (Math.abs(v * 2 - Math.round(v * 2)) < 1e-9) { // every 0.5 V for the surface
                for (double dist : SURFACE_DISTANCES) {
                    surface.add(new double[]{dist, v, DriveTimePredictor.timeTo(p, () -> rp.battery(vv), other, Axis.FORWARD, dist).timeS()});
                }
            }
        }
        ctx.log.printf("  top speed %.2f m/s at 12.0 V vs %.2f m/s at 13.5 V%n", interp(topVsV, 12.0), interp(topVsV, 13.5));

        save.param("drive.max_speed_mps", fwd.topSpeed());
        save.param("drive.max_strafe_speed_mps", strafe.topSpeed());
        save.param("drive.max_turn_rate_dps", turnDps);
        save.param("drive.max_accel_mps2", fwd.peakAccel());
        save.param("drive.max_decel_mps2", decel);
        save.param("drive.time_to_max_speed_s", fwd.timeTo95());
        save.param("drive.peak_current_a", fwd.peakCurrent());
        save.param("drive.cruise_current_a", fwd.cruiseCurrent());
        save.curve("drive.speed_vs_time", speedVsTime);
        save.curve("drive.current_vs_time", currentVsTime);
        save.curve("drive.current_vs_speed", currentVsSpeed);
        save.curve("drive.time_to_distance", tFwd);
        save.curve("drive.strafe_time_to_distance", tStrafe);
        save.curve("drive.time_to_turn", tTurn);
        save.curve("drive.top_speed_vs_battery_v", topVsV);
        save.curve("drive.max_accel_vs_battery_v", accelVsV);
        save.surface("drive.time_to_distance_vs_battery", surface);

        HtmlReport r = new HtmlReport("Drive sim");
        Reports.provenance(r, rp, ctx);
        r.heading("Summary");
        r.table(Arrays.asList("quantity", "value", "unit"), Arrays.asList(
                Reports.row("top speed forward", fwd.topSpeed(), "m/s"),
                Reports.row("top speed strafing", strafe.topSpeed(), "m/s"),
                Reports.row("top turn rate", turnDps, "deg/s"),
                Reports.row("peak acceleration", fwd.peakAccel(), "m/s^2"),
                Reports.row("peak deceleration", decel, "m/s^2"),
                Reports.row("time to 95% speed", fwd.timeTo95(), "s"),
                Reports.row("peak drive current (4 motors)", fwd.peakCurrent(), "A"),
                Reports.row("drive current at top speed", fwd.cruiseCurrent(), "A"),
                Reports.row("theoretical free speed (no losses)", p.theoreticalTopSpeed(), "m/s")));
        r.heading("Full power from rest");
        r.chart(new SvgChart("Speed", "time (s)", "m/s").zeroBased().add(Reports.series("forward", speedVsTime)));
        r.chart(new SvgChart("Drive current", "time (s)", "A").zeroBased().add(Reports.series("4 motors", currentVsTime)));
        r.chart(new SvgChart("Battery voltage", "time (s)", "V").add(Reports.series("bus", busVsTime)));
        r.chart(new SvgChart("Current vs speed", "speed (m/s)", "A").zeroBased().add(Reports.series("full power", currentVsSpeed)));
        r.heading("Stop-to-stop move times (physics limit)");
        r.chart(new SvgChart("Time to drive a distance", "distance (m)", "s").zeroBased()
                .add(Reports.series("forward", tFwd)).add(Reports.series("strafe", tStrafe)));
        r.chart(new SvgChart("Time to turn in place", "angle (deg)", "s").zeroBased().add(Reports.series("turn", tTurn)));
        r.heading("Battery effect");
        r.chart(new SvgChart("Top speed vs battery", "resting voltage (V)", "m/s").add(Reports.series("top speed", topVsV)));
        r.chart(new SvgChart("Peak acceleration vs battery", "resting voltage (V)", "m/s^2").add(Reports.series("peak accel", accelVsV)));
        SvgChart sc = new SvgChart("Move time by distance and battery", "resting voltage (V)", "s").zeroBased();
        for (double dist : SURFACE_DISTANCES) {
            List<double[]> line = new ArrayList<>();
            for (double[] q : surface) if (q[0] == dist) line.add(new double[]{q[1], q[2]});
            sc.add(Reports.series(dist + " m", line));
        }
        r.chart(sc);
        r.write(ctx.outDir.resolve("drive/report.html"));
        fwd.rec().write(ctx.outDir.resolve("drive/full_power_forward.csv"));
        ctx.log.println("  report: " + ctx.outDir.resolve("drive/report.html"));
        save.report();
    }

    static double interp(List<double[]> pts, double x) {
        for (int i = 1; i < pts.size(); i++) {
            double[] a = pts.get(i - 1), b = pts.get(i);
            if (x >= a[0] && x <= b[0]) return a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1]);
        }
        return Double.NaN;
    }
}
