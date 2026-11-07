package org.athletech.sim.config;

import java.util.Locale;

public enum SourceKind {
    MEASURED(5),
    GAME_MANUAL(5),
    MANUAL(4),
    SIM(3),
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
