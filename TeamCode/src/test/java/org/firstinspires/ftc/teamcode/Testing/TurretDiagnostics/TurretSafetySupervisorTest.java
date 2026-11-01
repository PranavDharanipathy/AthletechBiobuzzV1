package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class TurretSafetySupervisorTest {

    /** 10 ticks per degree keeps the arithmetic readable. */
    static TurretDiagnosticSettings.Builder valid() {
        TurretDiagnosticSettings.Builder b = new TurretDiagnosticSettings.Builder();
        b.ticksPerDegree = 10;
        b.minDeg = -90;
        b.maxDeg = 90;
        b.stopMarginDeg = 10;
        b.maxCommand = 0.2;
        b.decelDegPerS2 = 500;
        b.maxSpeedDegPerS = 200;
        b.maxLoopGapMs = 100;
        return b;
    }

    private static final long MS = 1_000_000L;
    private TurretSafetySupervisor sup;
    private TurretDiagnosticSettings settings;
    private long t;

    @Before public void setUp() {
        sup = new TurretSafetySupervisor();
        settings = valid().build();
        t = 1_000 * MS;
    }

    /** Confirms everything, samples a still turret at {@code deg}, arms and engages the deadman. */
    private void runningAt(double deg) {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(deg);
        sample(deg);
        assertTrue(sup.arm(DiagnosticActivity.OPEN_LOOP, settings, false, t));
        assertEquals(StopReason.NONE, sup.check(0, true, t));
        assertTrue(sup.isRunning());
    }

    private void sample(double deg) {
        t += 10 * MS;
        sup.observe(t, (int) Math.round(deg * 10));
    }

    @Test public void passiveNeverArms() {
        assertNotNull(sup.armRefusal(DiagnosticActivity.PASSIVE, settings, false, t));
    }

    @Test public void armingNeedsEveryConfirmation() {
        sample(0);
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t).contains("settings"));
        sup.confirmSettings(settings);
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t).contains("reference"));
        sup.confirmReference(0);
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, true, t).contains("deadman"));
        assertNull(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t));
    }

    @Test public void defaultsAreNotArmable() {
        TurretDiagnosticSettings defaults = new TurretDiagnosticSettings.Builder().build();
        sup.confirmSettings(defaults);
        sup.confirmReference(0);
        sample(0);
        String refusal = sup.armRefusal(DiagnosticActivity.OPEN_LOOP, defaults, false, t);
        assertNotNull(refusal);
        assertTrue(refusal, refusal.contains("TICKS_PER_DEGREE"));
    }

    @Test public void editAfterConfirmationRefusesArming() {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(0);
        TurretDiagnosticSettings.Builder edited = valid();
        edited.maxCommand = 0.3;
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, edited.build(), false, t).contains("re-confirm"));
    }

    @Test public void refusesOutsideSafeZoneAndWhileMoving() {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(85); // safe max is 90 - 10
        sample(85);
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t).contains("safe zone"));
        sample(0);
        sample(1); // 1 deg in 10 ms = 100 deg/s
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t).contains("moving"));
    }

    @Test public void refusesStaleSample() {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(0);
        assertTrue(sup.armRefusal(DiagnosticActivity.OPEN_LOOP, settings, false, t + 150 * MS).contains("stale"));
    }

    @Test public void armedWaitsForDeadmanThenTimesOut() {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(0);
        sample(0);
        assertTrue(sup.arm(DiagnosticActivity.OPEN_LOOP, settings, false, t));
        assertEquals(StopReason.NONE, sup.check(0, false, t));
        assertTrue(sup.isArmed());
        assertFalse(sup.isRunning());
        long late = t + (long) ((TurretSafetySupervisor.ARM_WAIT_TIMEOUT_S + 1) * 1e9);
        sup.observe(late, 0);
        assertEquals(StopReason.ARM_TIMEOUT, sup.check(0, false, late));
        assertFalse(sup.isArmed());
    }

    @Test public void deadmanReleaseStops() {
        runningAt(0);
        sample(0);
        assertEquals(StopReason.DEADMAN_RELEASED, sup.check(0, false, t));
        assertFalse(sup.isArmed());
        assertFalse(sup.lastStop().fault);
    }

    @Test public void readFailureStops() {
        runningAt(0);
        sup.markReadFailed();
        assertEquals(StopReason.FEEDBACK_READ_FAILED, sup.check(0.1, true, t));
    }

    @Test public void staleSampleAndLongLoopGapStop() {
        runningAt(0);
        assertEquals(StopReason.STALE_FEEDBACK, sup.check(0, true, t + 150 * MS));

        runningAt(0);
        t += 150 * MS;
        sup.observe(t, 0);
        assertEquals(StopReason.STALE_FEEDBACK, sup.check(0, true, t));
    }

    @Test public void implausibleJumpStops() {
        runningAt(0);
        sample(20); // 20 deg in 10 ms = 2000 deg/s
        assertEquals(StopReason.IMPLAUSIBLE_JUMP, sup.check(0.1, true, t));
    }

    @Test public void overspeedStops() {
        runningAt(0);
        sample(-2.5); // 250 deg/s, below the implausible threshold, toward the far bound
        assertEquals(StopReason.OVERSPEED, sup.check(0.1, true, t));
    }

    @Test public void outsideBoundsStopsOnEngage() {
        sup.confirmSettings(settings);
        sup.confirmReference(0);
        sample(75);
        sample(75);
        assertTrue(sup.arm(DiagnosticActivity.OPEN_LOOP, settings, false, t));
        // Armed but not engaged: someone pushes the turret slowly past the bound.
        for (double deg = 75; deg <= 95; deg += 0.25) {
            sample(deg);
            assertEquals(StopReason.NONE, sup.check(0, false, t));
        }
        assertEquals(StopReason.OUTSIDE_BOUNDS, sup.check(0, true, t));
        assertFalse(sup.isArmed());
    }

    @Test public void slowDriftPastBoundStops() {
        runningAt(0);
        // Creep outward at 50 deg/s: predicted stop eventually crosses the margin, before the bound.
        StopReason reason = StopReason.NONE;
        double deg = 0;
        while (reason == StopReason.NONE && deg < 100) {
            deg += 0.5;
            sample(deg);
            reason = sup.check(0.05, true, t);
        }
        assertEquals(StopReason.PREDICTED_STOP_BEYOND_MARGIN, reason);
        assertTrue("stopped before the safe max", deg <= settings.safeMaxDeg());
    }

    @Test public void movingAwayFromNearBoundIsAllowed() {
        runningAt(79);
        sample(78.5);
        assertEquals(StopReason.NONE, sup.check(-0.05, true, t));
    }

    @Test public void frozenEncoderUnderCommandStops() {
        runningAt(0);
        StopReason reason = StopReason.NONE;
        for (int i = 0; i < 100 && reason == StopReason.NONE; i++) {
            sample(0);
            reason = sup.check(0.15, true, t); // above half of max command 0.2
        }
        assertEquals(StopReason.ENCODER_FROZEN, reason);
    }

    @Test public void lowCommandWithoutMotionIsNotAFault() {
        runningAt(0);
        for (int i = 0; i < 100; i++) {
            sample(0);
            assertEquals(StopReason.NONE, sup.check(0.05, true, t)); // below breakaway is legitimate
        }
    }

    @Test public void sustainedOpposingMotionStops() {
        runningAt(0);
        StopReason reason = StopReason.NONE;
        double deg = 0;
        for (int i = 0; i < 100 && reason == StopReason.NONE; i++) {
            deg -= 0.1; // 10 deg/s against a positive command
            sample(deg);
            reason = sup.check(0.15, true, t);
        }
        assertEquals(StopReason.COMMAND_OPPOSES_MOTION, reason);
    }

    @Test public void faultIsShownUntilRearmed() {
        runningAt(0);
        sup.markReadFailed();
        sup.check(0, true, t);
        assertTrue(sup.lastStop().fault);
        sample(0);
        sample(0);
        assertTrue(sup.arm(DiagnosticActivity.OPEN_LOOP, settings, false, t));
        assertEquals(StopReason.NONE, sup.lastStop());
    }

    @Test public void confirmationsRefusedWhileArmed() {
        runningAt(0);
        assertFalse(sup.confirmReference(500));
        assertFalse(sup.confirmSettings(valid().build()));
        assertEquals(0, sup.referenceTicks());
    }

    @Test public void predictedStopIncludesLatencyAndBraking() {
        runningAt(0);
        sample(1); // 100 deg/s
        double expected = 1 + 100 * (2 * 0.010) + 100.0 * 100 / (2 * 500);
        assertEquals(expected, sup.predictedStopDeg(), 1e-6);
    }
}
