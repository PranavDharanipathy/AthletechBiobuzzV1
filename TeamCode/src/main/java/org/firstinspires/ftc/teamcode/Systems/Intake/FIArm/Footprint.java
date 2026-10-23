package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class Footprint {

    public abstract Vec2 getReference();

    public abstract Vec2 support(double dx, double dy);

    public static class Circle extends Footprint {
        private final Vec2 center;
        private final double radius;

        public Circle(Vec2 center, double radius) {
            if (radius < 0.0) throw new IllegalArgumentException("radius must be >= 0");
            this.center = center;
            this.radius = radius;
        }

        public Vec2 getCenter() {
            return center;
        }

        public double getRadius() {
            return radius;
        }

        @Override
        public Vec2 getReference() {
            return center;
        }

        @Override
        public Vec2 support(double dx, double dy) {
            double n = Math.hypot(dx, dy);
            if (n < 1e-12) return center;
            return new Vec2(center.getX() + radius * dx / n, center.getY() + radius * dy / n);
        }
    }

    public static class Polygon extends Footprint {
        private final List<Vec2> points;
        private final Vec2 reference;

        public Polygon(List<Vec2> points) {
            if (points.isEmpty()) throw new IllegalArgumentException("polygon needs at least one point");
            this.points = new ArrayList<Vec2>(points);
            double sumX = 0.0;
            double sumY = 0.0;
            for (Vec2 p : this.points) {
                sumX += p.getX();
                sumY += p.getY();
            }
            this.reference = new Vec2(sumX / this.points.size(), sumY / this.points.size());
        }

        public List<Vec2> getPoints() {
            return Collections.unmodifiableList(points);
        }

        @Override
        public Vec2 getReference() {
            return reference;
        }

        @Override
        public Vec2 support(double dx, double dy) {
            Vec2 best = points.get(0);
            double bestValue = best.getX() * dx + best.getY() * dy;
            for (int i = 1; i < points.size(); i++) {
                Vec2 p = points.get(i);
                double v = p.getX() * dx + p.getY() * dy;
                if (v > bestValue) {
                    bestValue = v;
                    best = p;
                }
            }
            return best;
        }
    }
}
