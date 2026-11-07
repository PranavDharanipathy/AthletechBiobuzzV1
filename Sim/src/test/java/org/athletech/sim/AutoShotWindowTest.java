package org.athletech.sim;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.AutoShotWindow;
import org.junit.Test;
import static org.junit.Assert.*;

public class AutoShotWindowTest {

    private AutoShotWindow window() {

        AutoShotWindow w = new AutoShotWindow();
        w.start(0, .5, 2, .1, .15);
        return w;
    }

    @Test public void neverOpensBeforeContinuousReadyDwell() {

        AutoShotWindow w = window();
        w.update(.02, true);
        w.update(.08, true);
        assertFalse(w.firing());
        w.update(.1, false);
        w.update(.12, true);
        w.update(.2, true);
        assertFalse(w.firing());
        w.update(.24, true);
        assertTrue(w.firing());
    }

    @Test public void countsOnlyActualOpenIntervalsAcrossRecovery() {

        AutoShotWindow w = window();
        w.update(.01, true);
        w.update(.12, true);
        assertTrue(w.firing());
        w.update(.22, false);
        assertEquals(.1, w.fedSeconds(), 1e-12);
        assertFalse(w.firing());
        w.update(.32, false);
        assertEquals(.1, w.fedSeconds(), 1e-12);
        w.update(.42, true);
        w.update(.53, true);
        for (int i = 1; i <= 5; i++) w.update(.53 + i * .1, true);
        assertEquals(AutoShotWindow.State.DONE, w.state());
        assertFalse(w.firing());
    }

    @Test public void neverReadyTimesOutWithoutOpening() {

        AutoShotWindow w = window();
        for (int i = 1; i <= 21; i++) w.update(i * .1, false);
        assertEquals(AutoShotWindow.State.TIMED_OUT, w.state());
        assertEquals(0, w.fedSeconds(), 0);
    }

    @Test public void timeoutEndsAnIncompleteBurst() {

        AutoShotWindow w = window();
        for (int i = 1; i < 19; i++) w.update(i * .1, false);
        w.update(1.9, true);
        w.update(2.01, true);
        assertEquals(AutoShotWindow.State.TIMED_OUT, w.state());
        assertFalse(w.firing());
    }

    @Test public void longLoopAndInvalidClockCloseAndLatchFault() {

        for (double time : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -.1, .5}) {
            AutoShotWindow w = window();
            w.update(.01, true);
            w.update(.12, true);
            w.update(time, true);
            assertEquals(AutoShotWindow.State.FAULT, w.state());
            assertFalse(w.firing());
            w.update(.6, true);
            assertFalse(w.firing());
        }
    }

    @Test public void nextShotResetsAllProgressAndReadiness() {

        AutoShotWindow w = window();
        w.update(.01, true);
        w.update(.12, true);
        w.update(.22, true);
        w.start(5, .5, 2, .1, .15);
        assertFalse(w.firing());
        assertEquals(0, w.fedSeconds(), 0);
        w.update(5.01, true);
        assertFalse(w.firing());
    }

    @Test public void invalidDurationsFailBeforeActuation() {

        for (double time : new double[] {Double.NaN, 0, -1, Double.POSITIVE_INFINITY, 2}) {
            try {
                new AutoShotWindow().start(0, time, 2, .1, .15);
                fail();
            }
            catch (IllegalArgumentException expected) {}
        }
    }
}
