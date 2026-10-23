package org.firstinspires.ftc.teamcode.Systems.Intake.FIArm;

public abstract class Extension {

    public static class HeightCap extends Extension {
        private final double z;

        public HeightCap(double z) {

            if (z < 0d) throw new IllegalArgumentException("height cap must be >= 0");

            this.z = z;
        }

        public double getZ() {
            return z;
        }
    }

    public static class Slant extends Extension {
        private final double x;
        private final double y;
        private final double z;

        public Slant(double x, double y, double z) {

            if (z <= 0d) throw new IllegalArgumentException("slant target height must be > 0");
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getZ() {
            return z;
        }
    }
}
