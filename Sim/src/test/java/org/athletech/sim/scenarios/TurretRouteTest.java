package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class TurretRouteTest {

    //20280512-221735-RedTeleOp: turret at -126, relocalize at the GARDEN, aim snaps to 130. It went for -200 then swung a full turn
    @Test
    public void relocalizationGoesStraightToTheNewAim() {

        TurretRoute route = new TurretRoute();
        double t = 0;
        for (int i = 0; i < 20; i++) {
            route.update(-126, 0, -126, -200, 200, 5, t);
            t += 0.03;
        }
        for (int i = 0; i < 40; i++) {
            double tt = route.update(130.2, 0, -126, -200, 200, 5, t);
            assertEquals(130.2, tt, 1e-9);
            assertFalse(route.hasSwung());
            t += 0.03;
        }
    }

    //the closed-form pick against trying every turn, turret anywhere (even past the ends on the hard-limit margin)
    @Test
    public void pickMatchesTryingEveryTurn() {

        java.util.Random rng = new java.util.Random(1);
        for (int i = 0; i < 200000; i++) {
            double aim = rng.nextDouble() * 360 - 180, now = rng.nextDouble() * 440 - 220;
            double best = Double.NaN;
            for (int k = -3; k <= 3; k++) {
                double c = aim + 360 * k;
                if (c < -200 || c > 200) continue;
                if (best != best || Math.abs(c - now) < Math.abs(best - now)) best = c;
            }
            assertEquals(best, new TurretRoute().update(aim, 0, now, -200, 200, 5, 0), 1e-9);
        }
    }

    //a spin still follows round past +-180 instead of jumping back
    @Test
    public void smoothSpinStillFollowsRound() {

        TurretRoute route = new TurretRoute();
        double aim = 150, t = 0, tt = 0;
        for (int i = 0; i < 15; i++) {
            tt = route.update(aim, 100, tt, -200, 200, 5, t);
            aim = aim + 3 > 180 ? aim + 3 - 360 : aim + 3;
            t += 0.03;
        }
        assertEquals(192, tt, 1e-9);
    }
}
