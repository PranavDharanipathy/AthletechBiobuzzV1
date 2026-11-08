package org.firstinspires.ftc.teamcode.Utility.Blaze;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.BlazeConstants;

import java.lang.reflect.Field;

import dev.anygeneric.blazeftc.BlazeDummyPlug;
import kotlin.Unit;

public final class BlazePinpointLocalizer implements Localizer {

    private final Localizer sdk;
    private final double refX, refY;
    private volatile int seq;
    private volatile long sampleNs;
    private volatile double rx, ry, rh, rvx, rvy, rw;
    private volatile double scale = Double.NaN;
    private double ox, oy, oh, rawX, rawY, rawH;
    private MotionState state;
    public long samples, sdkReads;
    public boolean usingSdk = true;
    public double sampleAgeSeconds = Double.NaN;

    private BlazePinpointLocalizer(Localizer sdk, double refXmm, double refYmm) {

        this.sdk = sdk;
        refX = refXmm;
        refY = refYmm;
        state = sdk.state();
        Pose p = state.pose();
        rawX = p.x();
        rawY = p.y();
        rawH = p.heading();
    }

    static BlazePinpointLocalizer attach(Follower follower, final BlazeRing ring) throws Exception {

        Localizer old = follower.localizer;
        if (old instanceof BlazePinpointLocalizer) return (BlazePinpointLocalizer) old;
        if (!(old instanceof PinpointLocalizer)) throw new IllegalStateException("follower's localizer isn't a Pinpoint");
        Field pf = PinpointLocalizer.class.getDeclaredField("pinpoint");
        pf.setAccessible(true);
        GoBildaPinpointDriver pinpoint = (GoBildaPinpointDriver) pf.get(old);
        pinpoint.update();
        final BlazePinpointLocalizer b = new BlazePinpointLocalizer(old, pinpoint.getPosX(DistanceUnit.MM), pinpoint.getPosY(DistanceUnit.MM));
        BlazeDummyPlug.engagePinpointAcceleration(pinpoint, data -> {
            b.accept(data.getXPosition(), data.getYPosition(), data.getDirection(), data.getXVelocity(), data.getYVelocity(), data.getAngVelocity());
            ring.push(System.nanoTime(), data.getXPosition(), data.getYPosition(), data.getDirection(), data.getXVelocity(), data.getYVelocity(), data.getAngVelocity());
            return Unit.INSTANCE;
        });
        Field lf = Follower.class.getField("localizer");
        lf.setAccessible(true);
        lf.set(follower, b);
        return b;
    }

    private void accept(double x, double y, double h, double vx, double vy, double w) {

        if (scale != scale) scale = units(x, y);
        seq++;
        rx = x;
        ry = y;
        rh = h;
        rvx = vx;
        rvy = vy;
        rw = w;
        sampleNs = System.nanoTime();
        seq++;
    }

    private double units(double x, double y) {

        if (Math.hypot(refX, refY) < 20) return BlazeConstants.PINPOINT_RAW_MM ? 25.4 : 1;
        double asMm = Math.hypot(x - refX, y - refY), asIn = Math.hypot(x - refX / 25.4, y - refY / 25.4);
        return asMm <= asIn ? 25.4 : 1;
    }

    public double unitsPerInch() {
        return scale;
    }

    @Override
    public void update() {

        long now = System.nanoTime();
        double x = 0, y = 0, h = 0, vx = 0, vy = 0, w = 0;
        long t = 0;
        boolean got = false;
        for (int tries = 0; tries < 4 && !got; tries++) {
            int s0 = seq;
            if ((s0 & 1) != 0) continue;
            x = rx;
            y = ry;
            h = rh;
            vx = rvx;
            vy = rvy;
            w = rw;
            t = sampleNs;
            got = s0 == seq && t != 0;
        }
        double k = scale;
        sampleAgeSeconds = got ? (now - t) * 1e-9 : Double.NaN;
        if (got && k == k && sampleAgeSeconds <= BlazeConstants.PINPOINT_STALE_SECONDS) {
            usingSdk = false;
            samples++;
            rawX = x / k;
            rawY = y / k;
            rawH = h;
            vx /= k;
            vy /= k;
        }
        else {
            usingSdk = true;
            sdkReads++;
            sdk.update();
            MotionState s = sdk.state();
            rawX = s.pose().x();
            rawY = s.pose().y();
            rawH = s.pose().heading();
            vx = s.velocity().vx;
            vy = s.velocity().vy;
            w = s.velocity().omega;
        }
        double c = Math.cos(oh), sn = Math.sin(oh);
        Pose pose = new Pose(c * rawX - sn * rawY + ox, sn * rawX + c * rawY + oy, normalize(rawH + oh));
        state = MotionState.ofVelocity(pose, new Velocity(c * vx - sn * vy, sn * vx + c * vy, w));
    }

    @Override
    public void setPose(Pose pose) {

        oh = pose.heading() - rawH;
        double c = Math.cos(oh), sn = Math.sin(oh);
        ox = pose.x() - (c * rawX - sn * rawY);
        oy = pose.y() - (sn * rawX + c * rawY);
        state = state.withPose(pose);
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {

        ox = oy = oh = 0;
        sdk.reset();
    }

    private static double normalize(double a) {

        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
