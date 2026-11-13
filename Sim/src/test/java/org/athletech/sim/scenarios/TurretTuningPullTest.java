package org.athletech.sim.scenarios;

import org.athletech.sim.tuning.TuningAnalyzer;
import org.junit.Test;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

//a finished Turret Autotuner result.csv lands in TunedConstants.TURRET_*, a stopped one doesn't
public class TurretTuningPullTest {

    @Test
    public void resultGoesIntoTunedConstants() throws Exception {

        File dir = Files.createTempDirectory("turretpull").toFile();
        File runs = new File(dir, "turret");
        write(new File(runs, "20280511-100000/result.csv"), "key,value\nstatus,done\nticks_per_degree,68.27\ngain,10500\nkinetic_friction,2.1\nbreakaway_friction,2.4\nkp_far,4.2\nkp_close,7.1\nki_far,15\nki_close,18\nkd_far,0.3\nkd_close,0.5\nk_hold_volts,0.4\nfriction_deadband_degrees,0.8\nmax_i,1500\nstribeck_degrees_per_second,0.2\nscore_before,1.1\nscore_after,0.8\n");
        write(new File(runs, "20280511-110000/result.csv"), "key,value\nstatus,stopped\ngain,1\n");
        File out = new File(dir, "TunedConstants.java");
        TuningAnalyzer.main(new String[] {new File(dir, "none").getPath(), out.getPath(), new File(dir, "REPORT.md").getPath(), runs.getPath(), new File(dir, "none").getPath()});
        String s = new String(Files.readAllBytes(out.toPath()), StandardCharsets.UTF_8);
        assertTrue(s, s.contains("TURRET_GAIN = 10500;"));
        assertTrue(s, s.contains("TURRET_KP_CLOSE = 7.1;"));
        assertTrue(s, s.contains("TURRET_STRIBECK_DEGREES_PER_SECOND = 0.2;"));
        assertTrue(s, s.contains("TURRET_AUTOTUNE = \"20280511-100000\";"));
        assertFalse(s, s.contains("TURRET_GAIN = 1;"));
        assertFalse(s, s.contains("TURRET_GAIN = 2;"));
    }

    static void write(File f, String text) throws Exception {

        f.getParentFile().mkdirs();
        Files.write(f.toPath(), text.getBytes(StandardCharsets.UTF_8));
    }
}
