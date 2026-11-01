package org.athletech.sim;

import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.utility.Vec3;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.junit.Assume.assumeTrue;

public class ProjectileRobustnessTest {
    private final Projectile.Ball ball = new Projectile.Ball("pollen", 0.02, 0.071, 0.5, 0.1);
    private final Aperture target = Aperture.circleFacingUp("cell", new Vec3(0, 0, 1), 0.5);
    private final Vec3 position = new Vec3(0, 0, 2), velocity = new Vec3(0.1, 0, 0);

    @Test public void invalidTimestepsAndBallPropertiesAreRejected() {
        invalid(() -> fly(Double.NaN));
        invalid(() -> fly(-0.001));
        // Avoid calling the old zero-dt infinite loop; zero is covered after the finite-dt cases.
        invalid(() -> fly(0));
        invalid(() -> new Projectile.Ball("bad", 0, 0.071, 0.5, 0));
        invalid(() -> new Projectile.Ball("bad", 0.02, Double.POSITIVE_INFINITY, 0.5, 0));
        invalid(() -> new Projectile.Ball("bad", 0.02, 0.071, -1, 0));
    }

    @Test public void timeoutDoesNotIntegratePastTheRequestedHorizon() {
        Aperture far = Aperture.circleFacingUp("far", new Vec3(100, 0, -100), 0.1);
        Projectile.Result result = Projectile.fly(ball, position, velocity, far, 1.2, 0.01, 0.025);
        assertEquals(Projectile.Outcome.TIMEOUT, result.outcome());
        assertEquals(0.025, result.timeS(), 1e-15);
    }

    @Test public void flightAllocationDoesNotGrowWithIntegrationSteps() {
        java.lang.management.ThreadMXBean raw = java.lang.management.ManagementFactory.getThreadMXBean();
        assumeTrue(raw instanceof com.sun.management.ThreadMXBean);
        com.sun.management.ThreadMXBean bean = (com.sun.management.ThreadMXBean) raw;
        assumeTrue(bean.isThreadAllocatedMemorySupported());
        bean.setThreadAllocatedMemoryEnabled(true);
        for (int i = 0; i < 100; i++) { fly(0.01); fly(0.001); }
        long coarse = allocated(bean, 0.01);
        long fine = allocated(bean, 0.001);
        System.out.println("Projectile bytes/flight: dt=0.01 " + coarse + ", dt=0.001 " + fine);
        assertTrue("per-flight allocations should be bounded, coarse=" + coarse + ", fine=" + fine,
                fine <= coarse * 1.5 + 2048);
    }

    @Test public void requestingATraceDoesNotChangeTheFlightSolution() {
        java.util.List<Vec3> trace = new java.util.ArrayList<>();
        Projectile.Result plain = fly(0.002);
        Projectile.Result traced = Projectile.fly(ball, position, velocity, target, 1.2, 0.002, 4, trace);
        assertEquals(plain.outcome(), traced.outcome());
        assertEquals(plain.timeS(), traced.timeS(), 0);
        assertEquals(plain.endPoint(), traced.endPoint());
        assertTrue(trace.size() > 100);
        assertEquals(position, trace.get(0));
    }

    private long allocated(com.sun.management.ThreadMXBean bean, double dt) {
        long id = Thread.currentThread().getId();
        long start = bean.getThreadAllocatedBytes(id);
        for (int i = 0; i < 20; i++) assertTrue(fly(dt).made());
        return (bean.getThreadAllocatedBytes(id) - start) / 20;
    }
    private Projectile.Result fly(double dt) {
        return Projectile.fly(ball, position, velocity, target, 1.2, dt, 4);
    }
    private static void invalid(Runnable operation) {
        try { operation.run(); }
        catch (IllegalArgumentException expected) { return; }
        fail("invalid flight input accepted");
    }
}
