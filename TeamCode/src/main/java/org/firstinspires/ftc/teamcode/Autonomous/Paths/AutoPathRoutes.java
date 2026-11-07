package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/// user's visualizer export (2026-11-07), points exactly as given, mirrored with mirrorX(70.75): x -> 141.5 - x,
/// heading -> 180 - heading. Every move is the mirror image of the drawing (path 1 goes to the robot's left)
public final class AutoPathRoutes {

    private static final PoseFactory P = PoseFactory.degrees().mirrorX(70.75);
    public static final Pose START = P.of(31.4072, 8.2495, 90);
    public static final Pose ONE = P.of(43.2487, 12.0376, 90);
    public static final Pose TWO_START = P.of(43.2487, 12.0376, 90);
    public static final Pose TWO = P.of(8.9248, 10.2542, 270);
    public static final Pose TWO_CONTROL = P.of(23.4838, 45.5865, 0);
    public static final Pose THREE_START = P.of(8.9248, 10.2542, 270);
    public static final Pose THREE = P.of(54.2865, 123.8186, 300);
    public static final Pose THREE_CONTROL = P.of(31.6945, 122.2396, 0);
    public static final Pose FOUR_START = P.of(54.2865, 123.8186, 300);
    public static final Pose FOUR = P.of(59.0665, 32.0101, 270);
    public static final Pose FOUR_CONTROL = P.of(65.0907, 118.0772, 0);
    public static final Pose FIVE = P.of(58.3463, 121.8874, 270);
    public static final Pose SIX = P.of(8.3808, 58.6368, 270);
    public static final Pose SIX_CONTROL = P.of(33.5008, 103.3373, 0);
    public static final Pose SEVEN = P.of(8.8433, 94.5704, 270);

    /// where each path ends, in order
    public static final Pose[] ENDS = {ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN};

    private AutoPathRoutes() {}

    public static Path path1() { return Paths.line(START, ONE).constant(ONE); }
    public static Path path2() { return Paths.curve(TWO_START, TWO_CONTROL, TWO).linear(TWO_START, TWO); }
    public static Path path3() { return Paths.curve(THREE_START, THREE_CONTROL, THREE).linear(THREE_START, THREE); }
    public static Path path4() { return Paths.curve(FOUR_START, FOUR_CONTROL, FOUR).linear(FOUR_START, FOUR); }
    public static Path path5() { return Paths.line(FOUR, FIVE).constant(FIVE); }
    public static Path path6() { return Paths.curve(FIVE, SIX_CONTROL, SIX).constant(SIX); }
    public static Path path7() { return Paths.line(SIX, SEVEN).constant(SEVEN); }

    public static Path[] paths() {
        return new Path[]{path1(), path2(), path3(), path4(), path5(), path6(), path7()};
    }
}
