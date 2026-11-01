package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

/** Columns of every turret diagnostic recording, in file order. Units are in the names. */
public enum Col {
    T_S("t_s"),
    LOOP_DT_MS("loop_dt_ms"),
    /** {@link DiagnosticActivity#code} actually in effect for this row. */
    ACTIVITY("activity"),
    /** {@link DiagnosticActivity#code} the operator has selected. */
    SELECTED_MODE("selected_mode"),
    /** Sub-plan: open loop 0=schedule 1=jog; controller 0=target schedule 1=hold reference. */
    PLAN("plan"),
    ARMED("armed"),
    DEADMAN("deadman"),
    RUN_ID("run_id"),
    /** Open-loop step index, or controller phase (-1 = holding start position). */
    STEP_INDEX("step_index"),
    /** {@link OpenLoopSchedule.Kind#code}; -1 outside open loop. */
    STEP_KIND("step_kind"),
    /** Command requested by the active source before the guard. */
    CMD_REQUESTED("cmd_requested"),
    CMD_APPLIED_L("cmd_applied_left"),
    CMD_APPLIED_R("cmd_applied_right"),
    /** 1 when the guard clamped the requested command to MAX_COMMAND. */
    SATURATED("saturated"),
    TICKS("ticks"),
    REF_TICKS("ref_ticks"),
    /** Degrees from reference using the operator-confirmed scale (NaN until confirmed). */
    POS_DEG("pos_deg"),
    /** Degrees from reference using the documented 68.2667 ticks/deg. */
    POS_DEG_DOCUMENTED("pos_deg_documented_scale"),
    /** Degrees from reference using baseline ShooterConstants 65.738 ticks/deg. */
    POS_DEG_BASELINE("pos_deg_baseline_scale"),
    /** Differenced velocity, confirmed scale, no filtering. */
    VEL_DEG_S("vel_deg_s"),
    /** Hub-reported encoder velocity, ticks/s (hub's own window; compare against differenced). */
    VEL_HUB_TPS("vel_hub_ticks_s"),
    PRED_STOP_DEG("predicted_stop_deg"),
    BATTERY_V("battery_v"),
    /** Bulk read, encoder and battery read, monitor update, microseconds. */
    READ_US("read_us"),
    /** Operator input, safety check, command computation and servo writes, microseconds. */
    WRITE_US("write_us"),
    CTRL_TARGET_TICKS("ctrl_target_ticks"),
    CTRL_ERROR_TICKS("ctrl_error_ticks"),
    CTRL_KP("ctrl_kp"),
    CTRL_KI("ctrl_ki"),
    CTRL_KD("ctrl_kd"),
    CTRL_KHOLD("ctrl_khold"),
    CTRL_P("ctrl_p"),
    CTRL_I("ctrl_i"),
    CTRL_D("ctrl_d"),
    CTRL_F("ctrl_f"),
    /** TurretBase's own output after its voltage compensation, before the guard. */
    CTRL_POWER("ctrl_power"),
    /** dt TurretBase itself measured (its own timer), ms. */
    CTRL_DT_MS("ctrl_dt_ms"),
    EXTRA_DELAY_MS("extra_delay_ms"),
    HWMON_LOOP_MS("hwmon_loop_ms"),
    STOP_CODE("stop_code");

    public final String header;

    Col(String header) {
        this.header = header;
    }

    public static final int COUNT = values().length;
}
