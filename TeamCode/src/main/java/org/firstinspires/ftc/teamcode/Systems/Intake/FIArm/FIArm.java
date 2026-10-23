package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Systems.Field.FieldCollisionMap;

import java.util.ArrayList;
import java.util.List;

// robot frame: x left, y forward, z up
// angles: 0 = horizontal forward, -PI/2 = straight down, swept linearly start -> end
public class FIArm {

    private static final double MAX_PIECE = Math.PI / 2d;

    private final double xMin;
    private final double xMax;
    private final double pivotY;
    private final double pivotZ;
    private final double length;
    private final double startAngle;
    private final double endAngle;
    private final double clearance;

    public FIArm(double pivotX1, double pivotX2, double pivotY, double pivotZ, double length) {
        this(pivotX1, pivotX2, pivotY, pivotZ, length, 0d, -Math.PI / 2d, 0d);
    }

    public FIArm(double pivotX1, double pivotX2, double pivotY, double pivotZ, double length, double startAngle, double endAngle) {
        this(pivotX1, pivotX2, pivotY, pivotZ, length, startAngle, endAngle, 0d);
    }

    public FIArm(double pivotX1, double pivotX2, double pivotY, double pivotZ, double length, double startAngle, double endAngle, double clearance) {

        if (length < 0d) throw new IllegalArgumentException("length must be >= 0");
        if (clearance < 0d) throw new IllegalArgumentException("clearance must be >= 0");

        this.xMin = Math.min(pivotX1, pivotX2);
        this.xMax = Math.max(pivotX1, pivotX2);
        this.pivotY = pivotY;
        this.pivotZ = pivotZ;
        this.length = length;
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.clearance = clearance;
    }

    public boolean canDrop(Pose pose, FieldCollisionMap field) {

        double zLow = pivotZ - length - clearance - 1d;
        double zHigh = pivotZ + length + clearance + 1d;

        List<SectorPrism> sweep = sweepShapes(pose);

        for (Obstacle obstacle : field.all()) {

            ConvexShape shape = obstacle.toShape(zLow, zHigh);
            for (SectorPrism piece : sweep) {
                if (GJK.intersects(piece, shape)) return false;
            }
        }
        return true;
    }

    public boolean canDrop(double x, double y, double heading, FieldCollisionMap field) {
        return canDrop(new Pose(x, y, heading), field);
    }

    //1 sector can only convex up to 180 degrees, so the sweep is split into 90 degree pieces
    private List<SectorPrism> sweepShapes(Pose pose) {

        double lo = Math.min(startAngle, endAngle);
        double hi = Math.max(startAngle, endAngle);
        double span = Math.min(hi - lo, 2 * Math.PI);
        int pieces = Math.max(1, (int) Math.ceil(span / MAX_PIECE - 1e-9));
        double step = span / pieces;

        List<SectorPrism> shapes = new ArrayList<SectorPrism>(pieces);
        for (int i = 0; i < pieces; i++) {

            shapes.add(new SectorPrism(
                    pose, xMin, xMax, pivotY, pivotZ, length,
                    lo + i * step, lo + (i + 1) * step, clearance));
        }
        return shapes;
    }

    public double getPivotX1() {
        return xMin;
    }

    public double getPivotX2() {
        return xMax;
    }

    public double getPivotY() {
        return pivotY;
    }

    public double getPivotZ() {
        return pivotZ;
    }

    public double getLength() {
        return length;
    }

    public double getStartAngle() {
        return startAngle;
    }

    public double getEndAngle() {
        return endAngle;
    }

    public double getClearance() {
        return clearance;
    }
}
