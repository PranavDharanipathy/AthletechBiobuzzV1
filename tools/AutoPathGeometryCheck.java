import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.AutoPathRoutes;
import org.firstinspires.ftc.teamcode.Constants.AutoPathConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import java.util.Locale;

public final class AutoPathGeometryCheck {

    private static final double FRONT = 359 / 25.4;
    private static final double SIDE = FieldConstants.ROBOT_CENTER_TO_SIDE;
    //inferred from the user's unchanged heading-90 start touching y=0; not a measured rear dimension
    private static final double REAR = AutoPathRoutes.START.y();
    private static final double[] FORWARD = {FRONT, -REAR};
    private static final double[] LATERAL = {-SIDE, SIDE};

    public static void main(String[] args) {

        Path[] paths = {AutoPathRoutes.path1(), AutoPathRoutes.path2(), AutoPathRoutes.path3(), AutoPathRoutes.path4(), AutoPathRoutes.path5(), AutoPathRoutes.path6(), AutoPathRoutes.path7()};
        Pose[] ends = {AutoPathRoutes.ONE, AutoPathRoutes.TWO, AutoPathRoutes.THREE, AutoPathRoutes.FOUR, AutoPathRoutes.FIVE, AutoPathRoutes.SIX, AutoPathRoutes.SEVEN};
        Pose previous = AutoPathRoutes.START;
        double length = 0;
        double positionError = AutoPathConstants.POSITION_ERROR_INCHES;
        double angleError = Math.toRadians(AutoPathConstants.HEADING_ERROR_DEGREES);
        double[] errors = {-angleError, 0, angleError};
        for (int leg = 0; leg < paths.length; leg++) {
            Path path = paths[leg];
            require(path.get(0).distance(previous) < 1e-8, "Disconnected start on path " + (leg + 1));
            require(path.endPose().distance(ends[leg]) < 1e-8, "Wrong end on path " + (leg + 1));
            double minimum = 100, worstPerturbed = 100, hiveGap = Double.POSITIVE_INFINITY;
            Pose last = path.get(0);
            for (int sample = 0; sample <= 4000; sample++) {
                Pose p = path.get(sample / 4000.0);
                require(Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading()), "Nonfinite path");
                if (sample > 0) length += p.distance(last);
                last = p;
                double margin = fieldMargin(p, p.heading(), 0);
                minimum = Math.min(minimum, margin);
                require(margin >= -1e-7, "Nominal footprint leaves field on path " + (leg + 1));
                require(maxX(p, p.heading()) <= FieldConstants.FIELD_SIZE / 2, "Footprint crosses alliance split");
                //the wall-contact start cannot have an inflated clearance margin
                if (leg == 0 && p.y() - REAR < 3) continue;
                for (double error : errors) {
                    double heading = p.heading() + error;
                    double perturbed = fieldMargin(p, heading, positionError);
                    worstPerturbed = Math.min(worstPerturbed, perturbed);
                    require(perturbed > 0, "Assumed tracking errors reach a wall on path " + (leg + 1));
                    require(maxX(p, heading) + positionError < 72, "Assumed tracking errors cross alliance split");
                    if (maxY(p, heading) + positionError >= 45 && minY(p, heading) - positionError <= 99) {
                        double gap = 45 - maxX(p, heading) - positionError;
                        hiveGap = Math.min(hiveGap, gap);
                        require(gap > 0, "Assumed footprint enters conservative central review envelope");
                    }
                }
            }
            System.out.printf(Locale.US, "Path %d: wall clearance %.2f in; with assumed errors %.2f in; central envelope gap %s%n", leg + 1, minimum, worstPerturbed, Double.isFinite(hiveGap) ? String.format(Locale.US, "%.2f in", hiveGap) : "not traversed");
            previous = ends[leg];
        }
        Pose[] shots = {AutoPathRoutes.ONE, AutoPathRoutes.THREE, AutoPathRoutes.FIVE};
        double minRange = ShooterConstants.FLYWHEEL_DISTANCES.get(0), maxRange = ShooterConstants.FLYWHEEL_DISTANCES.get(ShooterConstants.FLYWHEEL_DISTANCES.size() - 1);
        for (int i = 0; i < shots.length; i++) {
            Pose p = shots[i];
            boolean north = p.y() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER;
            require(north == (i == 1), "Firing sides must alternate south/north/south");
            double minimum = 1000, maximum = 0;
            for (double dx : new double[] {-positionError, 0, positionError}) {
                for (double dy : new double[] {-positionError, 0, positionError}) {
                    for (double dh : errors) {
                        double d = range(new Pose(p.x() + dx, p.y() + dy, p.heading() + dh));
                        minimum = Math.min(minimum, d);
                        maximum = Math.max(maximum, d);
                        require(d >= minRange && d <= maxRange, "Shot outside calibrated distance-table domain");
                    }
                }
            }
            System.out.printf(Locale.US, "Shot %d: nominal %.2f in; assumed-error range %.2f..%.2f in%n", i + 1, range(p), minimum, maximum);
        }
        System.out.printf(Locale.US, "PASS: 28007 samples, approximate route %.1f in. Front %.3f, side %.3f, rear %.4f (rear inferred).%n", length, FRONT, SIDE, REAR);
        System.out.println("Error assumptions are not measured tracking guarantees. Start contact, flower retrieval, real 3-D geometry, pollen pickup and actual hive tips still require field verification.");
    }

    private static double range(Pose p) {

        double x = p.x() + ShooterConstants.TURRET_OFFSET_X * Math.cos(p.heading()) - ShooterConstants.TURRET_OFFSET_Y * Math.sin(p.heading());
        double y = p.y() + ShooterConstants.TURRET_OFFSET_X * Math.sin(p.heading()) + ShooterConstants.TURRET_OFFSET_Y * Math.cos(p.heading());
        boolean north = p.y() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER;
        return Math.hypot((north ? FieldConstants.RED_GOAL_CLOSE_X : FieldConstants.RED_GOAL_FAR_X) - x, (north ? FieldConstants.RED_GOAL_CLOSE_Y : FieldConstants.RED_GOAL_FAR_Y) - y);
    }

    private static double fieldMargin(Pose p, double heading, double error) {

        double margin = 1000;
        for (double f : FORWARD) for (double l : LATERAL) {
            double x = p.x() + f * Math.cos(heading) - l * Math.sin(heading);
            double y = p.y() + f * Math.sin(heading) + l * Math.cos(heading);
            margin = Math.min(margin, Math.min(Math.min(x, FieldConstants.FIELD_SIZE - x), Math.min(y, FieldConstants.FIELD_SIZE - y)) - error);
        }
        return margin;
    }

    private static double maxX(Pose p, double h) { return edge(p, h, true, true); }
    private static double minY(Pose p, double h) { return edge(p, h, false, false); }
    private static double maxY(Pose p, double h) { return edge(p, h, false, true); }

    private static double edge(Pose p, double h, boolean xAxis, boolean max) {

        double result = max ? -1000 : 1000;
        for (double f : FORWARD) for (double l : LATERAL) {
            double value = xAxis ? p.x() + f * Math.cos(h) - l * Math.sin(h) : p.y() + f * Math.sin(h) + l * Math.cos(h);
            result = max ? Math.max(result, value) : Math.min(result, value);
        }
        return result;
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
