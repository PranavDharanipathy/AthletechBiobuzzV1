package org.firstinspires.ftc.teamcode.Utility.Blaze;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

//Blaze's Pedro 3 localizer as is, plus setPose (it throws "not implemented"): a relocalization moves and turns everything after it
public final class BlazeLocalizer implements Localizer {

    private final Localizer blaze;
    private volatile double dx, dy, dh;

    BlazeLocalizer(Localizer blaze) {
        this.blaze = blaze;
    }

    @Override
    public MotionState state() {

        MotionState raw = blaze.state();
        Pose p = raw.pose();
        Velocity v = raw.velocity();
        double c = Math.cos(dh), s = Math.sin(dh);
        double h = dh, x = dx, y = dy;
        return MotionState.ofVelocity(new Pose(x + c * p.x() - s * p.y(), y + s * p.x() + c * p.y(), p.heading() + h), new Velocity(c * v.vx - s * v.vy, s * v.vx + c * v.vy, v.omega));
    }

    @Override
    public void setPose(Pose pose) {

        Pose p = blaze.state().pose();
        double h = pose.heading() - p.heading(), c = Math.cos(h), s = Math.sin(h);
        dx = pose.x() - (c * p.x() - s * p.y());
        dy = pose.y() - (s * p.x() + c * p.y());
        dh = h;
    }

    @Override
    public void update() {}

    @Override
    public void reset() {}
}
