package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class TurretDiagnosticSettingsTest {

    private static boolean mentions(List<String> problems, String text) {
        for (String p : problems) if (p.contains(text)) return true;
        return false;
    }

    @Test public void defaultsBlockEveryActiveModeButNotPassive() {
        TurretDiagnosticSettings d = new TurretDiagnosticSettings.Builder().build();
        assertTrue(d.problems(DiagnosticActivity.PASSIVE).isEmpty());
        List<String> p = d.problems(DiagnosticActivity.OPEN_LOOP);
        assertTrue(mentions(p, "TICKS_PER_DEGREE"));
        assertTrue(mentions(p, "MIN_DEG"));
        assertTrue(mentions(p, "MAX_COMMAND"));
        assertTrue(mentions(p, "STOP_MARGIN_DEG"));
        assertTrue(mentions(p, "DECEL"));
        assertTrue(mentions(p, "MAX_SPEED"));
        assertFalse(d.problems(DiagnosticActivity.CONTROLLER).isEmpty());
    }

    @Test public void validSettingsPass() {
        TurretDiagnosticSettings s = TurretSafetySupervisorTest.valid().build();
        assertEquals(s.problems(DiagnosticActivity.OPEN_LOOP).toString(), 0, s.problems(DiagnosticActivity.OPEN_LOOP).size());
        assertEquals(s.problems(DiagnosticActivity.CONTROLLER).toString(), 0, s.problems(DiagnosticActivity.CONTROLLER).size());
    }

    @Test public void boundsMustContainReferenceWithMargin() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        b.minDeg = 5;
        assertTrue(mentions(b.build().problems(DiagnosticActivity.OPEN_LOOP), "MIN_DEG < 0 < MAX_DEG"));
        b = TurretSafetySupervisorTest.valid();
        b.minDeg = -8; // margin 10 leaves no room on that side
        assertTrue(mentions(b.build().problems(DiagnosticActivity.OPEN_LOOP), "no room"));
    }

    @Test public void levelsAndTargetsAreBoundedByConfirmedLimits() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        b.openLoopLevels = new double[] {0.1, 0.3}; // max command 0.2
        assertTrue(mentions(b.build().problems(DiagnosticActivity.OPEN_LOOP), "0.300"));
        b = TurretSafetySupervisorTest.valid();
        b.controllerTargetsDeg = new double[] {85}; // safe max 80
        assertTrue(mentions(b.build().problems(DiagnosticActivity.CONTROLLER), "85.00"));
        b = TurretSafetySupervisorTest.valid();
        b.pulseS = 5;
        assertTrue(mentions(b.build().problems(DiagnosticActivity.OPEN_LOOP), "PULSE_S"));
    }

    @Test public void extraLoopDelayCannotDefeatStaleDetection() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        b.extraLoopDelayMs = 40;
        b.extraLoopJitterMs = 20; // 60 > 100 / 2
        assertTrue(mentions(b.build().problems(DiagnosticActivity.CONTROLLER), "MAX_LOOP_GAP_MS / 2"));
    }

    @Test public void sameAsIsNanSafeAndDetectsEdits() {
        TurretDiagnosticSettings a = new TurretDiagnosticSettings.Builder().build();
        assertTrue(a.sameAs(new TurretDiagnosticSettings.Builder().build()));
        TurretDiagnosticSettings.Builder b = new TurretDiagnosticSettings.Builder();
        b.openLoopLevels = new double[] {0.04};
        assertFalse(a.sameAs(b.build()));
    }

    @Test public void builderArraysAreCopied() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        double[] levels = {0.05};
        b.openLoopLevels = levels;
        TurretDiagnosticSettings s = b.build();
        levels[0] = 0.9;
        assertEquals(0.05, s.openLoopLevels[0], 0);
    }

    @Test public void scaleWarningOnlyWhenMatchingNeitherCandidate() {
        TurretDiagnosticSettings.Builder b = TurretSafetySupervisorTest.valid();
        b.ticksPerDegree = TurretDiagnosticSettings.DOCUMENTED_TICKS_PER_DEGREE;
        assertTrue(b.build().warnings().isEmpty());
        b.ticksPerDegree = TurretDiagnosticSettings.BASELINE_TICKS_PER_DEGREE;
        assertTrue(b.build().warnings().isEmpty());
        b.ticksPerDegree = 10;
        assertFalse(b.build().warnings().isEmpty());
    }

    @Test public void documentedScaleIsTheGearedQuadratureValue() {
        assertEquals(68.2667, TurretDiagnosticSettings.DOCUMENTED_TICKS_PER_DEGREE, 1e-4);
    }
}
