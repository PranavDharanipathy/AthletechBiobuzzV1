package org.athletech.sim;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.DcMotor;
import org.athletech.sim.physics.MotorSpec;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.robot.Turret;
import org.athletech.sim.utility.Vec3;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Physics models against hand-checkable answers. */
public class PhysicsTest {

    /** goBILDA 5203 312 rpm: 24.3 kg*cm stall, 9.2 A stall, 0.25 A free, 12 V. */
    static final MotorSpec YJ312 = MotorSpec.fromRpm(24.3 * Physics.KG_CM_TO_NM, 9.2, 312, 0.25, 12);

    @Test
    public void motorReproducesItsDatasheet() {
        DcMotor m = new DcMotor(YJ312);
        m.setDuty(1);
        double stallI = m.current(12, 0);
        assertEquals(9.2, stallI, 1e-9);
        assertEquals(YJ312.stallTorqueNm(), m.torque(stallI, 0), 1e-9);
        double free = YJ312.freeSpeedRadPerSec();
        double freeI = m.current(12, free);
        assertEquals(0.25, freeI, 1e-9);
        assertEquals(0, m.torque(freeI, free), 1e-9);
    }

    @Test
    public void motorBrakeFloatAndCurrentLimit() {
        DcMotor m = new DcMotor(YJ312);
        m.setDuty(0);
        m.setBrake(true);
        assertTrue("brake opposes motion", m.current(12, 10) < 0);
        m.setBrake(false);
        assertEquals(0, m.current(12, 10), 0);
        m.setDuty(1);
        m.setCurrentLimit(4);
        assertEquals(4, m.current(12, 0), 1e-12);
        assertEquals(4, m.batteryCurrent(4), 1e-12);
    }

    @Test
    public void batterySagsByIR() {
        Battery b = new Battery(13, 0.1, 0);
        assertEquals(12, b.solve(v -> 10), 1e-6);
        // a resistive load I = V/2 ohm: V = 13 - 0.1*V/2 -> V = 13/1.05
        assertEquals(13 / 1.05, b.solve(v -> v / 2), 1e-6);
    }

    @Test
    public void vacuumRangeMatchesFormula() {
        Projectile.Ball tiny = new Projectile.Ball("dot", 0.05, 0.0002, 0, 0);
        // target plane below the floor so the flight ends on the floor (an upward opening's plane
        // ends any flight that comes down through its height, scoring or not)
        Aperture far = Aperture.circleFacingUp("far", new Vec3(1000, 0, -5), 0.1);
        double v = 6, a = Math.toRadians(45);
        Projectile.Result r = Projectile.fly(tiny, new Vec3(0, 0, 0.0002), new Vec3(v * Math.cos(a), 0, v * Math.sin(a)), far,
                Physics.AIR_DENSITY, 0.0005, 5);
        assertEquals(v * v / Physics.G, r.endPoint().x(), 0.01);
    }

    @Test
    public void dragShortensTheShot() {
        Aperture far = Aperture.circleFacingUp("far", new Vec3(1000, 0, 0.5), 0.1);
        Vec3 v0 = new Vec3(5, 0, 5);
        double vac = Projectile.fly(new Projectile.Ball("b", 0.02, 0.07, 0, 0), new Vec3(0, 0, 0.3), v0, far).endPoint().x();
        double air = Projectile.fly(new Projectile.Ball("b", 0.02, 0.07, 0.5, 0), new Vec3(0, 0, 0.3), v0, far).endPoint().x();
        assertTrue("drag should shorten " + vac + " -> " + air, air < vac * 0.97);
    }

    @Test
    public void ballDroppedIntoAnOpeningScoresAndOneBesideItMisses() {
        Projectile.Ball b = new Projectile.Ball("b", 0.02, 0.07, 0.5, 0);
        Aperture up = Aperture.oriented("cell", new Vec3(0, 0, 1), 0, 0, 0.5, 0.35, false);
        assertTrue(Projectile.fly(b, new Vec3(0, 0, 2), new Vec3(0.01, 0, 0), up).made());
        assertTrue(!Projectile.fly(b, new Vec3(0.4, 0, 2), new Vec3(0.01, 0, 0), up).made());
        // a vertical opening facing +x, hit straight on
        Aperture wall = Aperture.oriented("wall", new Vec3(0, 0, 1), 0, Math.PI / 2, 0.5, 0.35, false);
        Projectile.Result r = Projectile.fly(b, new Vec3(2, 0, 1), new Vec3(-12, 0, 0.5), wall);
        assertTrue("outcome " + r.outcome(), r.made());
    }

    @Test
    public void ballComingUpThroughAnOpeningDoesNotScore() {
        Projectile.Ball b = new Projectile.Ball("b", 0.02, 0.07, 0.5, 0);
        Aperture up = Aperture.oriented("cell", new Vec3(0, 0, 1), 0, 0, 0.5, 0.35, false);
        Projectile.Result r = Projectile.fly(b, new Vec3(0, 0, 0.5), new Vec3(0.01, 0, 5), up);
        assertTrue("hits the underside: " + r.outcome(), !r.made() && r.outcome() == Projectile.Outcome.SHORT);
    }

    static MecanumDrive.Params drive(double mu, double crr, double eta) {
        return new MecanumDrive.Params(13, 0.4, 0.052, 0.36, 0.30, 1, YJ312, mu, eta, crr, 0, 1, true);
    }

    static double run(MecanumDrive d, Battery b, double seconds, double f, double s, double t) {
        for (double time = 0; time < seconds; time += 0.001) {
            d.drive(f, s, t);
            d.step(0.001, b, 0);
        }
        return d.speed();
    }

    @Test
    public void mecanumTopSpeedsAndTractionLimit() {
        // no losses, ideal 12 V battery: top speed = motor free speed * wheel radius
        MecanumDrive d = new MecanumDrive(drive(5, 0, 0.8));
        d.reset(0, 0, 0);
        double top = run(d, new Battery(12, 0, 0), 3, 1, 0, 0);
        assertEquals(drive(5, 0, 0.8).theoreticalTopSpeed(), top, 0.005 * top);
        // strafing: eta times as fast
        MecanumDrive s = new MecanumDrive(drive(5, 0, 0.8));
        s.reset(0, 0, 0);
        run(s, new Battery(12, 0, 0), 3, 0, 1, 0);
        assertEquals(0.8 * top, s.leftSpeed(), 0.01 * top);
        assertEquals(0, s.forwardSpeed(), 1e-6);
        // traction: first step acceleration is mu*g/sqrt(2)
        MecanumDrive t = new MecanumDrive(drive(0.5, 0, 0.8));
        t.reset(0, 0, 0);
        t.drive(1, 0, 0);
        t.step(0.001, new Battery(12, 0, 0), 0);
        assertEquals(0.5 * Physics.G / Math.sqrt(2), t.forwardSpeed() / 0.001, 0.01);
        // turning in place does not translate
        MecanumDrive r = new MecanumDrive(drive(5, 0.04, 0.8));
        r.reset(0, 0, 0);
        run(r, new Battery(12, 0, 0), 1, 0, 0, 1);
        assertEquals(0, Math.hypot(r.x(), r.y()), 1e-9);
        assertTrue(r.omega() > 1);
    }

    @Test
    public void lowerBatteryMeansSlowerRobot() {
        MecanumDrive a = new MecanumDrive(drive(0.9, 0.04, 0.8)), b = new MecanumDrive(drive(0.9, 0.04, 0.8));
        a.reset(0, 0, 0);
        b.reset(0, 0, 0);
        assertTrue(run(a, new Battery(13.5, 0.08, 0), 3, 1, 0, 0) > run(b, new Battery(11.8, 0.08, 0), 3, 1, 0, 0) * 1.1);
    }

    @Test
    public void flywheelSpinsUpAndShotEnergyBalances() {
        MotorSpec bare = MotorSpec.fromRpm(0.144, 9.2, 6000, 0.25, 12);
        Flywheel.Params p = new Flywheel.Params(0.0012, 0.048, 1, 2, bare, 0.005, 0.002, false);
        Flywheel f = new Flywheel(p);
        f.setTargetRpm(4000);
        Battery b = new Battery(13, 0.05, 0);
        for (int i = 0; i < 6000; i++) f.step(0.001, b, 0);
        assertEquals(4000, f.rpm(), 60);
        double w0 = Physics.rpmToRadPerSec(f.rpm());
        double v = f.fire(0.02, 0.5, 1.6);
        double w1 = Physics.rpmToRadPerSec(f.rpm());
        double lostKe = 0.5 * 0.0012 * (w0 * w0 - w1 * w1);
        assertEquals(1.6 * 0.5 * 0.02 * v * v, lostKe, 1e-9);
        assertTrue(v > 0.45 * w1 * 0.048 && v < 0.5 * w0 * 0.048 + 1e-9);
    }

    @Test
    public void turretTrapezoid() {
        Turret t = new Turret(360, 1800, 0.1, 270);
        // 180 deg: accel 0.2 s to full speed (36 deg), cruise 108 deg (0.3 s), decel 0.2 s
        assertEquals(0.7 + 0.1, t.timeToTurn(180), 1e-9);
        // 10 deg never reaches full speed: 2*sqrt(10/1800)
        assertEquals(2 * Math.sqrt(10.0 / 1800) + 0.1, t.timeToTurn(10), 1e-9);
    }
}
