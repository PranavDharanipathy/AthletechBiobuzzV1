package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPathRoutes {

    private static final PoseFactory P = PoseFactory.degrees();
    public static final Pose START = P.of(31.407, 8.249, 90);
    public static final Pose ONE = P.of(43.249, 12.038, 90);
    public static final Pose TWO = P.of(8.925, 10.254, 270);
    public static final Pose THREE = P.of(54.286, 123.819, 300);
    public static final Pose FOUR = P.of(59.067, 32.010, 270);
    public static final Pose FIVE = P.of(58.346, 121.887, 270);
    public static final Pose SIX = P.of(9.888, 58.637, 270);
    public static final Pose SEVEN = P.of(11.428, 94.993, 270);

    private AutoPathRoutes() {}

    public static Path path1() { return Paths.line(START, ONE).constant(ONE); }
    public static Path path2() { return Paths.curve(ONE, P.of(23.484, 45.586, 0), TWO).linear(ONE, TWO); }
    public static Path path3() { return Paths.curve(TWO, P.of(31.695, 122.240, 0), THREE).linear(TWO, THREE); }
    public static Path path4() { return Paths.curve(THREE, P.of(65.091, 118.077, 0), FOUR).linear(THREE, FOUR); }
    public static Path path5() { return Paths.line(FOUR, FIVE).constant(FIVE); }
    public static Path path6() { return Paths.curve(FIVE, P.of(33.501, 103.337, 0), SIX).constant(SIX); }
    public static Path path7() { return Paths.line(SIX, SEVEN).constant(SEVEN); }
}
