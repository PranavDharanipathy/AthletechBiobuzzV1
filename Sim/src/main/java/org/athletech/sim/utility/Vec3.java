package org.athletech.sim.utility;

/** Immutable 3D vector. Field frame: +z up. */
public final class Vec3 {
    private final double x;
    private final double y;
    private final double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Vec3)) return false;
        Vec3 o = (Vec3) other;
        return Double.compare(x, o.x) == 0
                && Double.compare(y, o.y) == 0
                && Double.compare(z, o.z) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(x, y, z);
    }

    @Override
    public String toString() {
        return "Vec3[x=" + x + ", y=" + y + ", z=" + z + "]";
    }

    public static final Vec3 ZERO = new Vec3(0, 0, 0);
    public static final Vec3 UP = new Vec3(0, 0, 1);

    public Vec3 plus(Vec3 o) { return new Vec3(x + o.x, y + o.y, z + o.z); }
    public Vec3 minus(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }
    public Vec3 times(double k) { return new Vec3(x * k, y * k, z * k); }
    public double dot(Vec3 o) { return x * o.x + y * o.y + z * o.z; }
    public Vec3 cross(Vec3 o) { return new Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
    public double norm() { return Math.sqrt(x * x + y * y + z * z); }

    public Vec3 normalized() {
        double n = norm();
        return n == 0 ? this : times(1 / n);
    }

    /** Horizontal part (z dropped). */
    public Vec2 xy() { return new Vec2(x, y); }
}
