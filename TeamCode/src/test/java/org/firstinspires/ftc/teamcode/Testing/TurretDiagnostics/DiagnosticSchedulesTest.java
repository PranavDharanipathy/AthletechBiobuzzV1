package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class DiagnosticSchedulesTest {

    private static final long S = 1_000_000_000L;

    private static OpenLoopSchedule schedule() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        b.openLoopLevels = new double[] {0.10, 0.05};
        b.pulseS = 0.2;
        b.holdS = 0.5;
        b.neutralS = 1.0;
        b.includeHolds = true;
        return OpenLoopSchedule.build(b.build());
    }

    @Test public void patternIsAscendingAlternatingWithNeutralBetweenMotion() {
        List<OpenLoopSchedule.Step> steps = schedule().steps();
        assertEquals(OpenLoopSchedule.Kind.NEUTRAL, steps.get(0).kind);
        double[] expectedPulses = {0.05, 0.05, -0.05, -0.05, 0.10, 0.10, -0.10, -0.10};
        for (int i = 0; i < expectedPulses.length; i++) {
            OpenLoopSchedule.Step motion = steps.get(1 + 2 * i);
            assertEquals(OpenLoopSchedule.Kind.PULSE, motion.kind);
            assertEquals(expectedPulses[i], motion.command, 0);
            assertEquals(OpenLoopSchedule.Kind.NEUTRAL, steps.get(2 + 2 * i).kind);
        }
        double net = 0;
        for (int i = 1; i < steps.size(); i++) {
            assertFalse("no two motion steps back to back",
                    steps.get(i).kind != OpenLoopSchedule.Kind.NEUTRAL && steps.get(i - 1).kind != OpenLoopSchedule.Kind.NEUTRAL);
            net += steps.get(i).command * steps.get(i).durationS;
        }
        assertEquals("command-time integral is balanced", 0, net, 1e-12);
        assertEquals(1 + 2 * 8 + 2 * 4, steps.size());
    }

    @Test public void advancesOnPlannedTimeAndCompletes() {
        OpenLoopSchedule s = schedule();
        long t0 = 100 * S;
        s.start(t0);
        assertEquals(0, s.update(t0), 0);
        assertEquals(0.05, s.update(t0 + (long) (1.05 * S)), 0); // after the 1 s lead neutral
        assertEquals(0, s.update(t0 + (long) (1.25 * S)), 0);
        assertEquals(0.05, s.update(t0 + (long) (2.25 * S)), 0);
        assertEquals(0, s.update(t0 + 1000 * S), 0);
        assertTrue(s.isComplete());
        assertFalse(s.isRunning());
    }

    @Test public void abortRestartsTheStepAfterANeutral() {
        OpenLoopSchedule s = schedule();
        long t0 = 100 * S;
        s.start(t0);
        assertEquals(0.05, s.update(t0 + (long) (1.1 * S)), 0); // mid first pulse
        int index = s.index();
        s.pause();
        assertEquals(0, s.update(t0 + 2 * S), 0);
        long t1 = 200 * S;
        s.start(t1);
        assertEquals("neutral first", 0, s.update(t1 + (long) (0.5 * S)), 0);
        assertEquals(OpenLoopSchedule.Kind.NEUTRAL, s.currentKind());
        assertEquals("then the full interrupted pulse", 0.05, s.update(t1 + (long) (1.05 * S)), 0);
        assertEquals(index, s.index());
        assertEquals(0.05, s.update(t1 + (long) (1.15 * S)), 0);
    }

    @Test public void controllerScheduleHoldsStartThenVisitsTargets() {
        ControllerTargetSchedule c = new ControllerTargetSchedule(new double[] {5, -5}, 2.0, false);
        long t0 = 10 * S;
        c.start(t0, 1.25);
        assertEquals(1.25, c.update(t0 + S), 0);
        assertEquals(-1, c.phase());
        assertEquals(5, c.update(t0 + 3 * S), 0);
        assertEquals(-5, c.update(t0 + 5 * S), 0);
        assertTrue(Double.isNaN(c.update(t0 + 7 * S)));
        assertTrue(c.isComplete());
    }

    @Test public void holdForeverStaysOnFirstTarget() {
        ControllerTargetSchedule c = new ControllerTargetSchedule(new double[] {0, 5}, 1.0, true);
        long t0 = 10 * S;
        c.start(t0, 3);
        assertEquals(3, c.update(t0), 0);
        assertEquals(0, c.update(t0 + 100 * S), 0);
        assertFalse(c.isComplete());
    }
}
