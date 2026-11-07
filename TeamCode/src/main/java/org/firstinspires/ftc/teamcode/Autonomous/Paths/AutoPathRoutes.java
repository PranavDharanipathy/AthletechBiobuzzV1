package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class AutoPathRoutes {

    private static final PoseFactory P = PoseFactory.degrees();
    public static final Pose START = P.of(31.4072, 8.2495, 90);
    public static final Pose ONE = P.of(20, 17, 90);
    public static final Pose TWO = P.of(10, 16.75, 270);
    public static final Pose THREE = P.of(20, 128, 300);
    public static final Pose FOUR = P.of(34, 32, 270);
    public static final Pose FIVE = P.of(17, 18, 270);
    public static final Pose SIX = P.of(11, 58.6368, 270);
    public static final Pose SEVEN = P.of(11, 94.5704, 270);

    private AutoPathRoutes() {}

    public static Path path1() { return Paths.line(START, ONE).constant(ONE); }
    public static Path path2() { return Paths.curve(ONE, P.of(32, 44, 0), TWO).linear(ONE, TWO); }
    public static Path path3() { return Paths.curve(TWO, P.of(31.6945, 122.2396, 0), THREE).linear(TWO, THREE); }
    public static Path path4() { return Paths.curve(THREE, P.of(32, 100, 0), FOUR).linear(THREE, FOUR); }
    public static Path path5() { return Paths.line(FOUR, FIVE).constant(FIVE); }
    public static Path path6() { return Paths.curve(FIVE, P.of(24, 54, 0), SIX).constant(SIX); }
    public static Path path7() { return Paths.line(SIX, SEVEN).constant(SEVEN); }
}
