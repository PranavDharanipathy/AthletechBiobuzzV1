package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.utility.Args;

import java.util.Random;

final class CodeLoop {

    final double meanSeconds, jitter;
    private final Random rnd;

    CodeLoop(double meanSeconds, double jitter, long seed) {

        this.meanSeconds = meanSeconds;
        this.jitter = Math.max(0, Math.min(0.9, jitter));
        rnd = new Random(seed);
    }

    double next() {
        return meanSeconds * (1 + jitter * (2 * rnd.nextDouble() - 1));
    }

    static CodeLoop from(Args args, RobotParams rp, long seed) {

        double ms = args != null && args.has("loop-ms") ? args.num("loop-ms", 20) : rp.opt("control.loop_time_ms", 20);
        double jitter = args != null ? args.num("loop-jitter", 0.3) : 0.3;
        return new CodeLoop(ms / 1000, jitter, seed);
    }

    String describe() {
        return String.format(java.util.Locale.ROOT, "%.0f ms +/-%.0f%%", 1000 * meanSeconds, 100 * jitter);
    }
}
