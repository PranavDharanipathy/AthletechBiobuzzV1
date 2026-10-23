package org.athletech.sim.utility;

/** Immutable 2D vector (meters unless stated otherwise). */
public final class Vec2 {
    private final double x;
    private final double y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() { return x; }
    public double y() { return y; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Vec2)) return false;
        Vec2 o = (Vec2) other;
        return Double.compare(x, o.x) == 0
                && Double.compare(y, o.y) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "Vec2[x=" + x + ", y=" + y + "]";
    }

    public static final Vec2 ZERO = new Vec2(0, 0);

    public static Vec2 polar(double length, double angleRad) {
        return new Vec2(length * Math.cos(angleRad), length * Math.sin(angleRad));
    }

    public Vec2 plus(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
    public Vec2 minus(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
    public Vec2 times(double k) { return new Vec2(x * k, y * k); }
    public double dot(Vec2 o) { return x * o.x + y * o.y; }
    public double cross(Vec2 o) { return x * o.y - y * o.x; }
    public double norm() { return Math.hypot(x, y); }
    public double angle() { return Math.atan2(y, x); }

    /** Rotates counter-clockwise by {@code angleRad}. */
    public Vec2 rotate(double angleRad) {
        double c = Math.cos(angleRad), s = Math.sin(angleRad);
        return new Vec2(c * x - s * y, s * x + c * y);
    }
}
