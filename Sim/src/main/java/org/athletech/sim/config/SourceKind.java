package org.athletech.sim.config;

import java.util.Locale;

/**
 * Where a value in the profile came from. Higher priority wins: a sim can never overwrite a value
 * someone measured, and real sim output replaces placeholder output.
 */
public enum SourceKind {
    /** Measured on the real robot (HardwareMonitor import, shot logs, a scale, a stopwatch). */
    MEASURED(5),
    /** Taken from the official game manual. */
    GAME_MANUAL(5),
    /** Typed in by a person (spec sheet, CAD, estimate). */
    MANUAL(4),
    /** Computed by a simulation in this module from real inputs. */
    SIM(3),
    /** Example/default value, or sim output computed from placeholder inputs. Never trust for strategy. */
    PLACEHOLDER(1);

    public final int priority;

    SourceKind(int priority) {
        this.priority = priority;
    }

    public String id() {
        return name().toLowerCase(Locale.ROOT);
    }

    public static SourceKind parse(String s) {
        if (s == null) return null;
        return SourceKind.valueOf(s.trim().toUpperCase(Locale.ROOT).replace('-', '_'));
    }
}
