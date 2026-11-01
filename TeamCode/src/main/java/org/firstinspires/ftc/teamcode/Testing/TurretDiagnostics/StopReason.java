package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

/**
 * Why command authority ended. Every reason stops both servos and disarms.
 * A fault ({@link #fault} = true) is also shown until the operator re-arms.
 */
public enum StopReason {
    NONE(0, false, "none"),
    // Normal endings.
    DEADMAN_RELEASED(1, false, "deadman released"),
    CANCELLED(2, false, "cancelled by operator"),
    SCHEDULE_COMPLETE(3, false, "schedule complete"),
    MODE_CHANGED(4, false, "mode changed"),
    OPMODE_STOP(5, false, "OpMode stopped"),
    ARM_TIMEOUT(6, false, "armed but deadman not pressed in time"),
    // Faults.
    FEEDBACK_READ_FAILED(10, true, "encoder read failed"),
    STALE_FEEDBACK(11, true, "loop gap too long: feedback stale"),
    IMPLAUSIBLE_JUMP(12, true, "implausible encoder jump"),
    OUTSIDE_BOUNDS(13, true, "position outside travel bounds"),
    PREDICTED_STOP_BEYOND_MARGIN(14, true, "predicted stop beyond travel margin"),
    OVERSPEED(15, true, "turret faster than max speed"),
    ENCODER_FROZEN(16, true, "commanded but encoder not moving"),
    COMMAND_OPPOSES_MOTION(17, true, "command opposes motion (polarity mismatch or severe overshoot)"),
    SETTINGS_CHANGED(18, true, "settings changed after confirmation"),
    RECORDING_FULL(19, true, "recording buffer full"),
    EXCEPTION(20, true, "exception in loop");

    public final int code;
    public final boolean fault;
    public final String text;

    StopReason(int code, boolean fault, String text) {
        this.code = code;
        this.fault = fault;
        this.text = text;
    }
}
