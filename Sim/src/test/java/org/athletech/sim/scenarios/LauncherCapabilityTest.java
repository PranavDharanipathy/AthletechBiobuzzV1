package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.SourceKind;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import static org.junit.Assert.*;

public class LauncherCapabilityTest {
    @Rule public final TemporaryFolder temporary = new TemporaryFolder();

    @Test public void pollenOnlyScenarioClearsStaleNectarSimulationButKeepsMeasurements() throws Exception {
        Path root = temporary.getRoot().toPath();
        try (ProfileDb db = ProfileDb.open(root.resolve("profile.db"))) {
            db.fillExamples();
            long example = db.addSource(SourceKind.PLACEHOLDER, "old example sim", null);
            long manual = db.addSource(SourceKind.MANUAL, "documented pollen-only shooter", null);
            long measured = db.addSource(SourceKind.MEASURED, "historical nectar observations", null);
            db.setParam("launcher.nectar.enabled", 0.0, null, null, manual, false);
            db.clear("ball.nectar.mass_kg");
            db.clear("launcher.nectar.exit_speed_ratio");
            db.clear("robot.max_nectar_held");
            db.setParam("launcher.nectar.max_range_m", 4.0, null, null, example, false);
            db.setCurve("launcher.nectar.make_prob_vs_distance", Collections.singletonList(new double[] {2, 0.9}), example, false);
            db.setCurve("launcher.nectar.measured_make_prob_vs_distance", Collections.singletonList(new double[] {2, 0.8}), measured, false);
            ByteArrayOutputStream bytes = new ByteArrayOutputStream();
            SimContext ctx = new SimContext(db, root.resolve("out"), true, false, 1, 1,
                    new PrintStream(bytes), "pollen-only regression with example physics");
            LauncherScenario.run(ctx);
            assertTrue(db.curve("launcher.pollen.make_prob_vs_distance").hasData());
            assertNull(db.param("launcher.nectar.max_range_m").value());
            assertFalse(db.curve("launcher.nectar.make_prob_vs_distance").hasData());
            assertEquals(0.8, db.curve("launcher.nectar.measured_make_prob_vs_distance").points().get(0)[1], 0);
            assertTrue(Files.isRegularFile(root.resolve("out/launcher/report.html")));
            assertFalse(bytes.toString("UTF-8").contains("NECTAR:"));
        }
    }
}
