package org.athletech.sim.field;

import org.athletech.sim.utility.Vec3;

/**
 * A scoring opening: a rectangle or circle in 3D. {@code normal} points out of the goal (toward
 * where balls come from). A ball scores when its center crosses the opening's plane going in, at
 * least one ball radius away from every edge.
 *
 * <p>{@link #oriented} covers every case: tilt 0 = opening faces straight up (a HIVE CELL in its
 * upward position, the top of a FLOWER), tilt 90 = opening is vertical and faces {@code facingYaw}.
 */
public final class Aperture {
    private final String name;
    private final Vec3 center;
    private final Vec3 normal;
    private final Vec3 widthAxis;
    private final Vec3 heightAxis;
    private final double width;
    private final double height;
    private final boolean circular;

    public Aperture(String name, Vec3 center, Vec3 normal, Vec3 widthAxis, Vec3 heightAxis, double width, double height, boolean circular) {
        this.name = name;
        this.center = center;
        this.normal = normal;
        this.widthAxis = widthAxis;
        this.heightAxis = heightAxis;
        this.width = width;
        this.height = height;
        this.circular = circular;
    }

    public String name() { return name; }
    public Vec3 center() { return center; }
    public Vec3 normal() { return normal; }
    public Vec3 widthAxis() { return widthAxis; }
    public Vec3 heightAxis() { return heightAxis; }
    public double width() { return width; }
    public double height() { return height; }
    public boolean circular() { return circular; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Aperture)) return false;
        Aperture o = (Aperture) other;
        return java.util.Objects.equals(name, o.name)
                && java.util.Objects.equals(center, o.center)
                && java.util.Objects.equals(normal, o.normal)
                && java.util.Objects.equals(widthAxis, o.widthAxis)
                && java.util.Objects.equals(heightAxis, o.heightAxis)
                && Double.compare(width, o.width) == 0
                && Double.compare(height, o.height) == 0
                && circular == o.circular;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(name, center, normal, widthAxis, heightAxis, width, height, circular);
    }

    @Override
    public String toString() {
        return "Aperture[name=" + name + ", center=" + center + ", normal=" + normal + ", widthAxis=" + widthAxis + ", heightAxis=" + heightAxis + ", width=" + width + ", height=" + height + ", circular=" + circular + "]";
    }

    /**
     * @param facingYawRad direction (field frame, CCW from +x) the opening faces horizontally
     * @param tiltFromUpRad 0 = faces up, pi/2 = faces horizontally toward {@code facingYawRad}
     * @param width         size along the horizontal edge (or the diameter if circular)
     * @param height        size along the other edge
     */
    public static Aperture oriented(String name, Vec3 center, double facingYawRad, double tiltFromUpRad,
                                    double width, double height, boolean circular) {
        double cy = Math.cos(facingYawRad), sy = Math.sin(facingYawRad);
        double st = Math.sin(tiltFromUpRad), ct = Math.cos(tiltFromUpRad);
        Vec3 n = new Vec3(cy * st, sy * st, ct).normalized();
        Vec3 u = new Vec3(-sy, cy, 0); // horizontal, along the opening's width
        Vec3 h = n.cross(u).normalized();
        return new Aperture(name, center, n, u, h, width, circular ? width : height, circular);
    }

    public static Aperture circleFacingUp(String name, Vec3 center, double diameter) {
        return oriented(name, center, 0, 0, diameter, diameter, true);
    }

    /** Signed distance of {@code p} from the opening plane; positive on the side balls come from. */
    public double signedDistance(Vec3 p) {
        return p.minus(center).dot(normal);
    }

    /** Coordinates of {@code p} in the opening plane: {along width, along height}. */
    public double[] local(Vec3 p) {
        Vec3 d = p.minus(center);
        return new double[]{d.dot(widthAxis), d.dot(heightAxis)};
    }

    /** Whether a ball of {@code radius} centered at in-plane point {@code p} fits through. */
    public boolean fits(Vec3 p, double radius) {
        double[] l = local(p);
        if (circular) return Math.hypot(l[0], l[1]) <= width / 2 - radius;
        return Math.abs(l[0]) <= width / 2 - radius && Math.abs(l[1]) <= height / 2 - radius;
    }

    /** Whether a ball of {@code radius} at in-plane point {@code p} touches the opening's outline or inside. */
    public boolean overlaps(Vec3 p, double radius) {
        double[] l = local(p);
        if (circular) return Math.hypot(l[0], l[1]) <= width / 2 + radius;
        return Math.abs(l[0]) <= width / 2 + radius && Math.abs(l[1]) <= height / 2 + radius;
    }

    /** Height of the opening's center above the tiles. */
    public double centerHeight() {
        return center.z();
    }
}
