package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.config.SourceKind;

import java.io.PrintStream;
import java.nio.file.Path;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.List;

/**
 * Everything a sim run needs: the profile, where to write reports, whether to save results back
 * into the profile, the random seed and Monte Carlo trial count.
 */
public final class SimContext {
    public final ProfileDb db;
    public final Path outDir;
    public final boolean save;
    public final boolean force;
    public final long seed;
    public final int trials;
    public final PrintStream log;
    public final String command;

    public SimContext(ProfileDb db, Path outDir, boolean save, boolean force, long seed, int trials, PrintStream log, String command) {
        this.db = db;
        this.outDir = outDir;
        this.save = save;
        this.force = force;
        this.seed = seed;
        this.trials = trials;
        this.log = log;
        this.command = command;
    }

    /** Writes results into the profile, all under one source row, and remembers what happened. */
    public final class Saver {
        private final RobotParams rp;
        private final String name;
        private long sourceId = -1;
        public final List<String> written = new ArrayList<>();
        public final List<String> kept = new ArrayList<>();

        Saver(RobotParams rp, String name) {
            this.rp = rp;
            this.name = name;
        }

        private long source() throws SQLException {
            if (sourceId < 0) sourceId = db.addSource(rp.outputKind(), name, rp.provenance(command));
            return sourceId;
        }

        private void record(String what, ProfileDb.Write w) {
            switch (w) {
                case WRITTEN:
                    written.add(what);
                    break;
                case KEPT_HIGHER_PRIORITY:
                    kept.add(what);
                    break;
                case UNKNOWN:
                    throw new IllegalStateException(what + " is not in the profile catalog");
            }
        }

        // A result the run could not produce (NaN / no points) is still written, as empty, so an
        // older run's numbers never survive next to the new ones.

        public void param(String key, double value) throws SQLException {
            if (!save) return;
            record(key, db.setParam(key, Double.isFinite(value) ? value : null, null, null, source(), force));
        }

        public void curve(String name, List<double[]> points) throws SQLException {
            if (!save) return;
            record(name, db.setCurve(name, points, source(), force));
        }

        public void surface(String name, List<double[]> points) throws SQLException {
            if (!save) return;
            record(name, db.setSurface(name, points, source(), force));
        }

        public void dist(String name, String kind, double mean, double std) throws SQLException {
            if (!save) return;
            boolean ok = Double.isFinite(mean) && Double.isFinite(std);
            record(name, db.setDist(name, ok ? kind : null, ok ? mean : null, ok ? std : null, null, null, null, null, source(), force));
        }

        public void action(String name, double meanS, double stdS, double success) throws SQLException {
            if (!save) return;
            boolean ok = Double.isFinite(meanS);
            record(name, db.setAction(name, ok ? meanS : null, ok ? stdS : null, null, ok ? success : null, source(), force));
        }

        public SourceKind kind() {
            return rp.outputKind();
        }

        /** Console summary after a run. */
        public void report() {
            if (!save) {
                log.println("  (not saved: add --save to store these results in " + db.path() + ")");
                return;
            }
            log.println("  saved " + written.size() + " entries to " + db.path() + " as " + kind().id());
            if (!kept.isEmpty()) log.println("  kept existing higher-priority values for: " + String.join(", ", kept) + " (use --force to overwrite)");
        }
    }

    public Saver saver(RobotParams rp, String name) {
        return new Saver(rp, name);
    }
}
