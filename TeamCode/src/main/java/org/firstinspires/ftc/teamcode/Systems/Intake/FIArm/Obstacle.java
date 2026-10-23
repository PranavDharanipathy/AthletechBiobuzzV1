package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;

import java.util.Arrays;
import java.util.List;

public class Obstacle {
    private final Footprint footprint;
    private final Extension extension;

    public Obstacle(Footprint footprint) {
        this(footprint, null);
    }

    public Obstacle(Footprint footprint, Extension extension) {
        this.footprint = footprint;
        this.extension = extension;
    }

    public Footprint getFootprint() {
        return footprint;
    }

    public Extension getExtension() {
        return extension;
    }

    public Obstacle withHeightCap(double z) {
        return new Obstacle(footprint, new Extension.HeightCap(z));
    }

    public Obstacle withSlantTo(double x, double y, double z) {
        return new Obstacle(footprint, new Extension.Slant(x, y, z));
    }

    public static Obstacle circle(double x, double y, double radius) {
        return new Obstacle(new Footprint.Circle(new Vec2(x, y), radius));
    }

    public static Obstacle rectangle(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
        return new Obstacle(new Footprint.Polygon(Arrays.asList(a, b, c, d)));
    }

    public static Obstacle wall(Vec2 a, Vec2 b) {
        return new Obstacle(new Footprint.Polygon(Arrays.asList(a, b)));
    }

    public static List<Obstacle> fieldBorder(Vec2 c1, Vec2 c2, Vec2 c3, Vec2 c4) {
        return Arrays.asList(wall(c1, c2), wall(c2, c3), wall(c3, c4), wall(c4, c1));
    }

    ConvexShape toShape(double zLow, double zHigh) {
        if (extension == null) {
            return new Prism(footprint, zLow, new Vec3(0.0, 0.0, zHigh - zLow));
        }
        if (extension instanceof Extension.HeightCap) {
            Extension.HeightCap cap = (Extension.HeightCap) extension;
            return new Prism(footprint, 0.0, new Vec3(0.0, 0.0, cap.getZ()));
        }
        if (extension instanceof Extension.Slant) {
            Extension.Slant slant = (Extension.Slant) extension;
            Vec2 r = footprint.getReference();
            return new Prism(footprint, 0.0,
                    new Vec3(slant.getX() - r.getX(), slant.getY() - r.getY(), slant.getZ()));
        }
        throw new IllegalStateException("unknown extension: " + extension);
    }
}
