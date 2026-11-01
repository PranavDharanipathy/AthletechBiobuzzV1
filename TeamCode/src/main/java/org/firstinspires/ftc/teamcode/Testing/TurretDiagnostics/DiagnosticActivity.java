package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

/**
 * Who has command authority over the turret servos.
 *
 * <p>Used twice in every recording: as the operator-selected mode, and as the activity that was
 * actually in effect for that row. While disarmed the activity is always {@link #PASSIVE}, whatever
 * mode is selected, so passive data is never labeled as a test.
 */
public enum DiagnosticActivity {
    /** No turret command is issued. The encoder is only observed. */
    PASSIVE(0, "passive"),
    /** The baseline {@code TurretBase} controller drives the servos, through the command guard. */
    CONTROLLER(1, "controller"),
    /** Bounded open-loop pulses, holds, or an operator jog, through the command guard. */
    OPEN_LOOP(2, "open_loop");

    public final int code;
    public final String label;

    DiagnosticActivity(int code, String label) {
        this.code = code;
        this.label = label;
    }

    public DiagnosticActivity next() {
        DiagnosticActivity[] all = values();
        return all[(ordinal() + 1) % all.length];
    }

    public DiagnosticActivity previous() {
        DiagnosticActivity[] all = values();
        return all[(ordinal() + all.length - 1) % all.length];
    }
}
