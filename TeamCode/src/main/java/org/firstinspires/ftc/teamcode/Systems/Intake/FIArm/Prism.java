package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;

public class Prism implements ConvexShape {

    private final Footprint footprint;
    private final double z0;
    private final Vec3 offset;

    public Prism(Footprint footprint, double z0, Vec3 offset) {

        this.footprint = footprint;
        this.z0 = z0;
        this.offset = offset;
    }

    @Override
    public Vec3 support(Vec3 d) {

        Vec2 p = footprint.support(d.getX(), d.getY());
        Vec3 base = new Vec3(p.getX(), p.getY(), z0);
        
        return d.dot(offset) > 0d ? base.plus(offset) : base;
    }
}
