package org.athletech.sim;

import org.athletech.sim.utility.Csv;
import org.athletech.sim.utility.Json;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.utility.Table1D;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class UtilTest {

    @Test
    public void tableInterpolatesAndClamps() {
        Table1D t = new Table1D(new double[]{2, 0, 1}, new double[]{4, 0, 1});
        assertEquals(0.5, t.at(0.5), 1e-12);
        assertEquals(2.5, t.at(1.5), 1e-12);
        assertEquals(4, t.at(9), 0);
        assertEquals(0, t.at(-3), 0);
    }

    @Test
    public void statsBasics() {
        Stats.Fit f = Stats.linearFit(new double[]{0, 1, 2, 3}, new double[]{1, 3, 5, 7});
        assertEquals(2, f.slope(), 1e-12);
        assertEquals(1, f.intercept(), 1e-12);
        assertEquals(0.5, Stats.normalCdf(0), 1e-7);
        assertEquals(0.975, Stats.normalCdf(1.959964), 1e-6);
        double[] w = Stats.wilson(50, 100);
        assertTrue(w[0] < 0.5 && w[1] > 0.5 && w[1] - w[0] < 0.21);
        assertEquals(5, Stats.percentile(new double[]{1, 9, 5}, 50), 0);
    }

    @Test
    public void jsonRoundTrip() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("a", Arrays.asList(1.5, "x\"y", true));
        m.put("b", Collections.singletonMap("c", Double.NaN));
        Object v = Json.parse(Json.write(m));
        assertEquals(1.5, ((List<?>) ((Map<?, ?>) v).get("a")).get(0));
        assertEquals("x\"y", ((List<?>) ((Map<?, ?>) v).get("a")).get(1));
        assertEquals(null, Json.obj(v, "b").get("c"));
    }

    @Test
    public void csvQuotesAndNumbers() {
        assertEquals(Arrays.asList("a", "b,c", "d\"e"), Csv.parseLine("a,\"b,c\",\"d\"\"e\""));
        assertEquals("0.1", Csv.fmt(0.1));
        assertEquals("3", Csv.fmt(3.0));
        assertEquals("12.3457", Csv.fmt(12.345678));
        assertEquals("", Csv.fmt(Double.NaN));
    }
}
