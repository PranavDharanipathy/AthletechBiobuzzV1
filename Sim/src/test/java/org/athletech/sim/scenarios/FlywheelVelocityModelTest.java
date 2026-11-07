package org.athletech.sim.scenarios;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelVelocityModel;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

//the match flywheel speed model: a least-squares fit through the tuned points, not a lookup
public class FlywheelVelocityModelTest {

    //the 2026-11 tuned table, fixed here so the test doesn't move when ShooterConstants is retuned
    static final java.util.List<Double> D = java.util.Arrays.asList(51.71503320693509, 56.29514009214838, 60.899456934836486, 68.99167060900542, 74.57368259902958, 80.60268703640344, 91.2495467341891, 100.4429764662134);
    static final java.util.List<Double> V = java.util.Arrays.asList(1300.0, 1330.0, 1380.0, 1440.0, 1455.0, 1485.0, 1590.0, 1665.0);

    //numpy.polyfit on the 2026-11 table: line 7.256031 d + 925.236135 (rms 12.6), quadratic 0.020858 d^2 + 4.099799 d + 1039.175660
    @Test
    public void fitsTheTunedPointsLikeNumpy() {

        List<Double> d = D, v = V;
        FlywheelVelocityModel line = new FlywheelVelocityModel();
        line.fit(d, v, 1);
        for (double x : new double[] {55, 75, 95}) assertEquals(7.256031 * x + 925.236135, line.evaluate(x), 1e-3);
        assertEquals(12.6, line.rms(), 0.1);
        FlywheelVelocityModel quad = new FlywheelVelocityModel();
        quad.fit(d, v, 2);
        for (double x : new double[] {55, 75, 95}) assertEquals(0.020858 * x * x + 4.099799 * x + 1039.175660, quad.evaluate(x), 0.05);
    }

    //past the tuned distances it follows the fit for the extrapolation margin, then holds
    @Test
    public void holdsPastTheExtrapolationMargin() {

        List<Double> d = D, v = V;
        FlywheelVelocityModel m = new FlywheelVelocityModel();
        double top = m.velocity(d, v, 1, 12, d.get(d.size() - 1) + 12);
        assertEquals(top, m.velocity(d, v, 1, 12, 300), 1e-9);
        double bottom = m.velocity(d, v, 1, 12, d.get(0) - 12);
        assertEquals(bottom, m.velocity(d, v, 1, 12, 0), 1e-9);
        assertTrue(bottom < v.get(0) && top > v.get(v.size() - 1));
    }

    //refits only when the points or the degree change; edits (Dashboard) take effect next call
    @Test
    public void refitsOnlyWhenThePointsChange() {

        List<Double> d = new ArrayList<>(Arrays.asList(50.0, 100.0)), v = new ArrayList<>(Arrays.asList(1300.0, 1700.0));
        FlywheelVelocityModel m = new FlywheelVelocityModel();
        assertEquals(1500, m.velocity(d, v, 1, 12, 75), 1e-9);
        for (int k = 0; k < 100; k++) m.velocity(d, v, 1, 12, 60 + k * 0.1);
        assertEquals(1, m.fits);
        v.set(1, 1800.0);
        assertEquals(1550, m.velocity(d, v, 1, 12, 75), 1e-9);
        assertEquals(2, m.fits);
    }

    //too few points for the degree drops the degree; none gives NaN (the Shooter then uses FLYWHEEL_SHOOT_VELOCITY); NaN points are skipped
    @Test
    public void degradesSafely() {

        FlywheelVelocityModel m = new FlywheelVelocityModel();
        assertEquals(1400, m.velocity(Arrays.asList(70.0), Arrays.asList(1400.0), 2, 12, 90), 1e-9);
        assertEquals(0, m.degree());
        assertTrue(Double.isNaN(m.velocity(new ArrayList<Double>(), new ArrayList<Double>(), 1, 12, 70)));
        assertEquals(1500, m.velocity(Arrays.asList(50.0, Double.NaN, 100.0), Arrays.asList(1300.0, 1450.0, 1700.0), 1, 12, 75), 1e-9);
    }
}
