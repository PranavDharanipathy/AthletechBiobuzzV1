package org.athletech.sim;

import org.athletech.sim.config.Catalog;
import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.ProfileExport;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.config.SourceKind;
import org.athletech.sim.input.MonitorImport;
import org.athletech.sim.input.ObservationImport;
import org.athletech.sim.scenarios.SimContext;
import org.athletech.sim.utility.Json;
import org.athletech.sim.utility.Rng;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** The profile database: catalog, priorities, curves, exports and imports. */
public class ProfileDbTest {
    Path dir;
    ProfileDb db;
    final PrintStream quiet = new PrintStream(new ByteArrayOutputStream());

    @Before
    public void open() throws Exception {
        dir = Files.createTempDirectory("profile-test");
        db = ProfileDb.open(dir.resolve("p.db"));
    }

    @After
    public void close() throws Exception {
        db.close();
        try (java.util.stream.Stream<Path> s = Files.walk(dir)) {
            s.sorted(java.util.Comparator.reverseOrder()).forEach(p -> p.toFile().delete());
        }
    }

    @Test
    public void newProfileHasEveryCatalogEntryEmptyExceptGameManual() throws Exception {
        assertEquals(Catalog.PARAMS.size(), db.params("").size());
        assertEquals(Catalog.CURVES.size(), db.curves("").size());
        assertNull(db.value("robot.mass_kg"));
        assertEquals(0.071, db.value("ball.pollen.diameter_m"), 1e-12);
        assertEquals(SourceKind.GAME_MANUAL, db.param("field.size_m").sourceKind());
        assertTrue(db.everything().stream().anyMatch(s -> s.name().equals("drive.time_to_distance") && !s.hasData()));
    }

    @Test
    public void reopeningKeepsValuesAndDoesNotDuplicate() throws Exception {
        long src = db.addSource(SourceKind.MANUAL, "test", null);
        db.setParam("robot.mass_kg", 12.5, null, 0.1, src, false);
        db.close();
        db = ProfileDb.open(dir.resolve("p.db"));
        assertEquals(12.5, db.value("robot.mass_kg"), 0);
        assertEquals(0.1, db.param("robot.mass_kg").std(), 0);
        assertEquals(Catalog.PARAMS.size(), db.params("").size());
    }

    @Test
    public void lowerPriorityNeverOverwritesHigher() throws Exception {
        long measured = db.addSource(SourceKind.MEASURED, "scale", null);
        long sim = db.addSource(SourceKind.SIM, "sim", null);
        long ph = db.addSource(SourceKind.PLACEHOLDER, "example", null);
        assertEquals(ProfileDb.Write.WRITTEN, db.setParam("drive.max_speed_mps", 1.5, null, null, ph, false));
        assertEquals(ProfileDb.Write.WRITTEN, db.setParam("drive.max_speed_mps", 1.7, null, null, sim, false));
        assertEquals(ProfileDb.Write.WRITTEN, db.setParam("drive.max_speed_mps", 1.65, null, null, measured, false));
        assertEquals(ProfileDb.Write.KEPT_HIGHER_PRIORITY, db.setParam("drive.max_speed_mps", 1.9, null, null, sim, false));
        assertEquals(1.65, db.value("drive.max_speed_mps"), 0);
        assertEquals(ProfileDb.Write.WRITTEN, db.setParam("drive.max_speed_mps", 1.9, null, null, sim, true));
        assertEquals(ProfileDb.Write.UNKNOWN, db.setParam("no.such_key", 1.0, null, null, sim, false));
    }

    @Test
    public void curvesSurfacesDistributionsActionsRoundTrip() throws Exception {
        long src = db.addSource(SourceKind.SIM, "t", null);
        db.setCurve("drive.time_to_distance", Arrays.asList(new double[]{1, 0.9}, new double[]{0.5, 0.6, 0.01, 10}), src, false);
        ProfileDb.CurveRow c = db.curve("drive.time_to_distance");
        assertEquals(2, c.points().size());
        assertEquals(0.5, c.points().get(0)[0], 0); // sorted by x
        assertEquals(10, c.points().get(0)[3], 0);
        db.setSurface("drive.time_to_distance_vs_battery", Arrays.asList(new double[]{1, 12, 0.95}, new double[]{1, 13, 0.9}), src, false);
        assertEquals(2, db.surface("drive.time_to_distance_vs_battery").points().size());
        db.setDist("control.loop_time_ms", "empirical", 10.0, 2.0, null, null, null, new double[]{9, 10, 11}, src, false);
        assertEquals(3, db.dist("control.loop_time_ms").samples().length);
        double s = db.distribution("control.loop_time_ms").sample(new Rng(1));
        assertTrue(s == 9 || s == 10 || s == 11);
        db.setAction("intake.nectar", 0.8, 0.2, null, 0.9, src, false);
        assertEquals(0.9, db.action("intake.nectar").successProb(), 0);
    }

    @Test
    public void examplesMakeEveryRequiredInputAvailableToTheSims() throws Exception {
        db.fillExamples();
        RobotParams rp = new RobotParams(db);
        assertNotNull(rp.drive());
        assertNotNull(rp.launcher());
        assertNotNull(rp.cell());
        assertNotNull(rp.odometry());
        rp.requireAll();
        assertEquals(SourceKind.PLACEHOLDER, rp.outputKind());
    }

    @Test
    public void missingInputsAreListed() throws Exception {
        RobotParams rp = new RobotParams(db);
        rp.drive();
        try {
            rp.requireAll();
            fail("expected missing inputs");
        } catch (RobotParams.MissingInputsException e) {
            assertTrue(e.keys.contains("robot.mass_kg"));
            assertTrue(e.getMessage().contains("profile set"));
        }
    }

    @Test
    public void jsonExportParsesBack() throws Exception {
        Path f = dir.resolve("p.json");
        ProfileExport.json(db, f);
        Object j = Json.parse(new String(Files.readAllBytes(f), StandardCharsets.UTF_8));
        Map<String, Object> p = Json.obj(j, "parameters");
        assertEquals(Catalog.PARAMS.size(), p.size());
        assertEquals(0.071, Json.num(Json.obj(p, "ball.pollen.diameter_m"), "value"), 0);
        ProfileExport.csv(db, dir.resolve("csv"));
        assertTrue(Files.size(dir.resolve("csv/parameters.csv")) > 1000);
        assertTrue(ProfileExport.docs().contains("`drive.time_to_distance`"));
    }

    @Test
    public void monitorImportFitsBatteryResistance() throws Exception {
        Path s = dir.resolve("HardwareMonitor/sessions/20261023-010000_Test");
        Files.createDirectories(s);
        StringBuilder csv = new StringBuilder("t_s,loop_ms,battery_v,total_current_a\n");
        Rng r = new Rng(5);
        for (int i = 0; i < 2000; i++) {
            double amps = 2 + 18 * Math.abs(Math.sin(i / 50.0));
            csv.append(i * 0.05).append(',').append(9 + r.gaussian()).append(',')
                    .append(13.0 - 0.08 * amps + 0.01 * r.gaussian()).append(',').append(amps).append('\n');
        }
        Files.write(s.resolve("system.csv"), csv.toString().getBytes(StandardCharsets.UTF_8));
        MonitorImport.run(db, dir.resolve("HardwareMonitor"), quiet, false, dir.resolve("out"));
        assertEquals(0.08, db.value("battery.internal_resistance_ohm"), 0.002);
        assertEquals(SourceKind.MEASURED, db.param("battery.internal_resistance_ohm").sourceKind());
        assertEquals(9, db.value("control.loop_time_ms"), 0.1);
        assertTrue(db.curve("battery.voltage_vs_current").hasData());
        assertTrue(db.dist("control.loop_time_ms").samples().length > 100);
    }

    @Test
    public void shotLogBecomesMeasuredMakeProbability() throws Exception {
        StringBuilder csv = new StringBuilder("ball,distance_m,result,target_rpm,actual_rpm\n");
        for (int i = 0; i < 20; i++) csv.append("pollen,2.0,").append(i < 15 ? "make" : "miss_short").append(",3000,").append(2990 + i).append('\n');
        Path f = dir.resolve("shots.csv");
        Files.write(f, csv.toString().getBytes(StandardCharsets.UTF_8));
        ObservationImport.importShots(db, f, quiet, false);
        // real shots go to the measured_* curve; the sim's own curve stays untouched
        ProfileDb.CurveRow c = db.curve("launcher.pollen.measured_make_prob_vs_distance");
        assertTrue(db.curve("launcher.pollen.make_prob_vs_distance").points().isEmpty());
        assertEquals(1, c.points().size());
        assertEquals(0.75, c.points().get(0)[1], 1e-12);
        assertEquals(SourceKind.MEASURED, c.sourceKind());
        assertEquals("normal", db.dist("launcher.rpm_error").kind());
        assertEquals(20, db.shots("pollen", "cell").size());
    }

    @Test
    public void poseRowWithoutPositionsIsRejected() throws Exception {
        Path f = dir.resolve("poses.csv");
        Files.write(f, "elapsed_s,true_x_m,true_y_m,est_x_m,est_y_m\n10,0.5,0.5,0.52,0.49\n20,0.5,,0.5,0.5\n".getBytes(StandardCharsets.UTF_8));
        try {
            ObservationImport.importPoses(db, f, quiet, false);
            fail("a pose row with an empty position must be rejected");
        } catch (java.io.IOException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("row 3"));
        }
    }

    @Test
    public void simSaverClearsResultsItCouldNotProduce() throws Exception {
        SimContext ctx = new SimContext(db, dir, true, false, 1, 10, quiet, "test");
        RobotParams rp = new RobotParams(db);
        SimContext.Saver first = ctx.saver(rp, "first run");
        first.param("launcher.pollen.max_range_m", 3.1);
        first.curve("launcher.pollen.make_prob_vs_distance", Arrays.asList(new double[]{1, 0.9}, new double[]{2, 0.7}));
        assertEquals(3.1, db.param("launcher.pollen.max_range_m").value(), 1e-12);
        SimContext.Saver second = ctx.saver(rp, "second run");
        second.param("launcher.pollen.max_range_m", Double.NaN);
        second.curve("launcher.pollen.make_prob_vs_distance", Collections.emptyList());
        assertNull("stale max range must not survive", db.param("launcher.pollen.max_range_m").value());
        assertTrue(db.curve("launcher.pollen.make_prob_vs_distance").points().isEmpty());
    }

    @Test
    public void clearWorksForEveryKind() throws Exception {
        db.setCurve("launcher.pollen.make_prob_vs_distance", Arrays.asList(new double[]{1, 0.5}), db.addSource(SourceKind.MANUAL, "t", ""), false);
        assertEquals("curve", db.clear("launcher.pollen.make_prob_vs_distance"));
        assertTrue(db.curve("launcher.pollen.make_prob_vs_distance").points().isEmpty());
        assertEquals("parameter", db.clear("robot.mass_kg"));
        assertNull(db.clear("no.such_thing"));
    }
}
