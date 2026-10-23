package org.firstinspires.ftc.teamcode.Utility.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.MonitorStore;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.MonitorWebServer;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Browses the HardwareMonitor database on the Driver Station. Works during INIT, so there is no
 * need to press play.
 *
 * <pre>
 * Session list   dpad up/down select   A open   Y reload   X twice = delete session
 * Session        dpad left/right page  dpad up/down scroll  B back
 *                on the Devices page:  A open the highlighted device
 * Device         dpad up/down scroll   dpad left/right previous/next device   B back
 * </pre>
 *
 * Clear everything with the "Hardware Monitor: Clear Database" OpMode. For charts and full
 * tables, open the web dashboard ({@link MonitorWebServer}) from a laptop on the robot's Wi-Fi.
 */
@TeleOp(name = "Hardware Monitor Viewer", group = "Diagnostics")
public class HardwareMonitorViewer extends LinearOpMode {

    private static final int VISIBLE_LINES = 17;
    private static final String[] PAGES = {"Overview", "Loop", "Power", "Hubs", "Motors", "Devices"};
    private static final int DEVICES_PAGE = 5;

    private enum Screen { LIST, SESSION, DEVICE }

    private Screen screen = Screen.LIST;
    private List<MonitorStore.SessionInfo> sessions = new ArrayList<>();
    private int selected;
    private boolean confirmDelete;
    private String status = "";
    private long databaseBytes;

    private JSONObject summary;
    private int page;
    private int scroll;
    private int deviceIndex;

    // previous button states for edge detection
    private boolean pUp, pDown, pLeft, pRight, pA, pB, pX, pY;

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        reload();
        while (!isStopRequested()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            handleInput(up && !pUp, down && !pDown, left && !pLeft, right && !pRight,
                    a && !pA, b && !pB, x && !pX, y && !pY);
            pUp = up;
            pDown = down;
            pLeft = left;
            pRight = right;
            pA = a;
            pB = b;
            pX = x;
            pY = y;

            render();
            telemetry.update();
            sleep(50);
        }
    }

    // ------------------------------------------------------------------ input

    private void handleInput(boolean up, boolean down, boolean left, boolean right,
                             boolean a, boolean b, boolean x, boolean y) {
        switch (screen) {
            case LIST:
                if (up) selected = Math.max(0, selected - 1);
                if (down) selected = Math.min(Math.max(0, sessions.size() - 1), selected + 1);
                if (up || down) confirmDelete = false;
                if (y) reload();
                if (x && !sessions.isEmpty()) {
                    if (confirmDelete) {
                        String id = sessions.get(selected).id;
                        status = MonitorStore.deleteSession(id) ? "Deleted " + id : "Could not delete " + id;
                        confirmDelete = false;
                        reload();
                    } else {
                        confirmDelete = true;
                    }
                }
                if (a && !sessions.isEmpty()) openSession(sessions.get(selected).id);
                break;
            case SESSION:
                if (left) {
                    page = (page + PAGES.length - 1) % PAGES.length;
                    scroll = 0;
                }
                if (right) {
                    page = (page + 1) % PAGES.length;
                    scroll = 0;
                }
                if (up) scroll = Math.max(0, scroll - 1);
                if (down) scroll++;
                if (a && page == DEVICES_PAGE && devices().length() > 0) {
                    deviceIndex = Math.min(scroll, devices().length() - 1);
                    screen = Screen.DEVICE;
                    scroll = 0;
                }
                if (b) {
                    screen = Screen.LIST;
                    summary = null;
                }
                break;
            case DEVICE:
                int n = devices().length();
                if (left && n > 0) {
                    deviceIndex = (deviceIndex + n - 1) % n;
                    scroll = 0;
                }
                if (right && n > 0) {
                    deviceIndex = (deviceIndex + 1) % n;
                    scroll = 0;
                }
                if (up) scroll = Math.max(0, scroll - 1);
                if (down) scroll++;
                if (b) {
                    screen = Screen.SESSION;
                    scroll = Math.max(0, deviceIndex);
                }
                break;
        }
    }

    private void reload() {
        sessions = MonitorStore.listSessions();
        databaseBytes = MonitorStore.totalSizeBytes();
        selected = Math.min(selected, Math.max(0, sessions.size() - 1));
    }

    private void openSession(String id) {
        try {
            summary = MonitorStore.readSummary(id);
            screen = Screen.SESSION;
            page = 0;
            scroll = 0;
            status = "";
        } catch (Exception e) {
            status = "Cannot open " + id + ": " + e.getMessage();
        }
    }

    // ------------------------------------------------------------------ rendering

    private void render() {
        List<String> lines = new ArrayList<>();
        String header;
        String footer;
        switch (screen) {
            case LIST:
                header = "HARDWARE MONITOR  " + sessions.size() + " sessions  "
                        + mb(databaseBytes) + " MB";
                renderList(lines);
                footer = confirmDelete ? "Press X again to DELETE the selected session"
                        : "A open  Y reload  X delete  (stored in " + MonitorStore.rootDir() + ")\n"
                        + "Charts + tables: http://192.168.43.1:8080" + MonitorWebServer.PATH
                        + " (phone RC: 192.168.49.1)";
                scroll = Math.max(0, selected - VISIBLE_LINES + 1);
                break;
            case SESSION:
                header = "< " + PAGES[page] + " >  " + obj(summary, "session").optString("id");
                renderPage(lines);
                footer = page == DEVICES_PAGE ? "A device  up/down select  left/right page  B back"
                        : "left/right page  up/down scroll  B back";
                break;
            default:
                JSONObject d = devices().optJSONObject(deviceIndex);
                header = "DEVICE " + (deviceIndex + 1) + "/" + devices().length() + "  "
                        + (d == null ? "" : d.optString("name"));
                renderDevice(d, lines);
                footer = "left/right device  up/down scroll  B back";
                break;
        }
        telemetry.addLine(header);
        if (!status.isEmpty()) telemetry.addLine(status);
        int max = Math.max(0, lines.size() - VISIBLE_LINES);
        if (screen != Screen.LIST) scroll = Math.min(scroll, max);
        int start = Math.min(scroll, max);
        for (int i = start; i < Math.min(lines.size(), start + VISIBLE_LINES); i++) {
            telemetry.addLine(lines.get(i));
        }
        if (lines.size() > VISIBLE_LINES) {
            telemetry.addLine(String.format(Locale.US, "-- %d-%d of %d --", start + 1,
                    Math.min(lines.size(), start + VISIBLE_LINES), lines.size()));
        }
        telemetry.addLine(footer);
    }

    private void renderList(List<String> lines) {
        if (sessions.isEmpty()) {
            lines.add("No sessions yet. Run an OpMode that uses Monitored*");
            lines.add("devices or calls HardwareMonitor.get().update().");
            return;
        }
        for (int i = 0; i < sessions.size(); i++) {
            MonitorStore.SessionInfo s = sessions.get(i);
            lines.add(String.format(Locale.US, "%s %-19s %s %6.1fs %6sms %5sV",
                    i == selected ? ">" : " ", trim(s.opMode, 19), shortStart(s.start),
                    s.durationS, num(s.avgLoopMs, 1), num(s.minBatteryV, 2))
                    + (s.complete ? "" : " *"));
        }
        boolean anyIncomplete = false;
        for (MonitorStore.SessionInfo s : sessions) anyIncomplete |= !s.complete;
        if (anyIncomplete) {
            lines.add("");
            lines.add("* = incomplete (still recording, or power was lost)");
        }
    }

    private void renderPage(List<String> lines) {
        JSONObject sess = obj(summary, "session");
        JSONObject loop = obj(summary, "loop");
        JSONObject power = obj(summary, "power");
        JSONObject jvm = obj(summary, "jvm");
        JSONObject mon = obj(summary, "monitor");
        switch (page) {
            case 0: {
                lines.add("OpMode   " + sess.optString("opMode") + (sess.optBoolean("complete") ? "" : "  (incomplete)"));
                lines.add("Started  " + sess.optString("start"));
                lines.add(String.format(Locale.US, "Length   %ss  (init %ss, run %ss)",
                        num(d(sess, "durationS"), 1), num(d(sess, "initS"), 1), num(d(sess, "runS"), 1)));
                lines.add(String.format(Locale.US, "Loop     %s ms avg  %s Hz  p95 %s  max %s",
                        num(d(loop, "avgMs"), 2), num(d(loop, "avgHz"), 0),
                        num(d(loop, "p95Ms"), 1), num(d(loop, "maxMs"), 1)));
                lines.add(String.format(Locale.US, "HW I/O   %s ms/loop (%s%%)  %s R %s W %s hit",
                        num(d(loop, "hwMsPerLoop"), 2), num(d(loop, "hwPctOfLoop"), 0),
                        num(d(loop, "readsPerLoop"), 1), num(d(loop, "writesPerLoop"), 1),
                        num(d(loop, "cacheHitsPerLoop"), 1)));
                JSONObject batt = obj(power, "batteryV");
                lines.add(String.format(Locale.US, "Battery  %s -> %s V  min %s  dips %d",
                        num(d(batt, "start"), 2), num(d(batt, "end"), 2), num(d(batt, "min"), 2),
                        power.optLong("lowVoltageEvents")));
                JSONObject cur = obj(power, "totalCurrentA");
                lines.add(String.format(Locale.US, "Current  avg %s A  max %s A",
                        num(d(cur, "avg"), 2), num(d(cur, "max"), 2)));
                lines.add(String.format(Locale.US, "Energy   %s Wh  %s mAh  peak %s W",
                        num(d(power, "energyWh"), 3), num(d(power, "chargeMah"), 0),
                        num(d(obj(power, "totalPowerW"), "max"), 0)));
                lines.add(String.format(Locale.US, "JVM      heap max %s/%s MB  CPU avg %s%%",
                        num(d(obj(jvm, "heapUsedMb"), "max"), 0), num(d(jvm, "heapMaxMb"), 0),
                        num(d(obj(jvm, "cpuPctOfOneCore"), "avg"), 0)));
                lines.add(String.format(Locale.US, "Monitor  %s ms/loop  %d samples  %d events (%d dropped)",
                        num(d(mon, "overheadMsPerLoop"), 3), mon.optLong("sampleReads"),
                        mon.optLong("eventsLogged"), mon.optLong("eventsDropped")));
                lines.add("Devices  " + devices().length() + "   hubs " + arr(summary, "hubs").length()
                        + "   motors " + arr(summary, "motors").length());
                if (!sess.optBoolean("lifecycleHook", true)) lines.add("! lifecycle hook was not active");
                break;
            }
            case 1: {
                lines.add("Loops     " + loop.optLong("count"));
                lines.add(String.format(Locale.US, "Average   %s ms  (%s Hz)", num(d(loop, "avgMs"), 3), num(d(loop, "avgHz"), 1)));
                lines.add(String.format(Locale.US, "Min/Max   %s / %s ms", num(d(loop, "minMs"), 3), num(d(loop, "maxMs"), 3)));
                lines.add(String.format(Locale.US, "Std dev   %s ms", num(d(loop, "stdMs"), 3)));
                lines.add(String.format(Locale.US, "p50/p90   %s / %s ms", num(d(loop, "p50Ms"), 2), num(d(loop, "p90Ms"), 2)));
                lines.add(String.format(Locale.US, "p95/p99   %s / %s ms", num(d(loop, "p95Ms"), 2), num(d(loop, "p99Ms"), 2)));
                lines.add(String.format(Locale.US, "HW I/O    %s ms/loop avg, %s max",
                        num(d(loop, "hwMsPerLoop"), 3), num(d(loop, "hwMsPerLoopMax"), 2)));
                lines.add(String.format(Locale.US, "HW share  %s%% of each loop", num(d(loop, "hwPctOfLoop"), 1)));
                lines.add(String.format(Locale.US, "Per loop  %s reads  %s writes  %s cache hits",
                        num(d(loop, "readsPerLoop"), 2), num(d(loop, "writesPerLoop"), 2),
                        num(d(loop, "cacheHitsPerLoop"), 2)));
                lines.add(String.format(Locale.US, "Monitor   %s ms total  %s ms/loop",
                        num(d(mon, "overheadMs"), 1), num(d(mon, "overheadMsPerLoop"), 3)));
                break;
            }
            case 2: {
                JSONObject batt = obj(power, "batteryV");
                lines.add(String.format(Locale.US, "Battery start  %s V", num(d(batt, "start"), 3)));
                lines.add(String.format(Locale.US, "Battery end    %s V", num(d(batt, "end"), 3)));
                lines.add(String.format(Locale.US, "Battery min    %s V   avg %s V", num(d(batt, "min"), 3), num(d(batt, "avg"), 3)));
                lines.add(String.format(Locale.US, "Dips below %s V: %d", num(d(power, "lowVoltageThresholdV"), 1),
                        power.optLong("lowVoltageEvents")));
                JSONObject cur = obj(power, "totalCurrentA");
                lines.add(String.format(Locale.US, "Total current  avg %s  max %s A", num(d(cur, "avg"), 2), num(d(cur, "max"), 2)));
                JSONObject pw = obj(power, "totalPowerW");
                lines.add(String.format(Locale.US, "Total power    avg %s  max %s W", num(d(pw, "avg"), 1), num(d(pw, "max"), 1)));
                lines.add(String.format(Locale.US, "Energy used    %s Wh", num(d(power, "energyWh"), 4)));
                lines.add(String.format(Locale.US, "Charge drawn   %s mAh", num(d(power, "chargeMah"), 1)));
                break;
            }
            case 3: {
                JSONArray hubs = arr(summary, "hubs");
                if (hubs.length() == 0) lines.add("No hub data (hardware map was never seen).");
                for (int i = 0; i < hubs.length(); i++) {
                    JSONObject h = hubs.optJSONObject(i);
                    lines.add(h.optString("name") + "  bulk " + h.optString("bulkCachingMode"));
                    lines.add(String.format(Locale.US, "  current avg %s  max %s A",
                            num(d(obj(h, "currentA"), "avg"), 2), num(d(obj(h, "currentA"), "max"), 2)));
                    lines.add(String.format(Locale.US, "  input   min %s  avg %s V   5V min %s",
                            num(d(obj(h, "inputV"), "min"), 2), num(d(obj(h, "inputV"), "avg"), 2),
                            num(d(obj(h, "auxV"), "min"), 2)));
                    lines.add(String.format(Locale.US, "  i2c %s A  gpio %s A  temp max %s C  noresp %d",
                            num(d(obj(h, "i2cA"), "avg"), 3), num(d(obj(h, "gpioA"), "avg"), 3),
                            num(d(obj(h, "tempC"), "max"), 1), h.optLong("notRespondingSamples")));
                }
                break;
            }
            case 4: {
                JSONArray motors = arr(summary, "motors");
                if (motors.length() == 0) lines.add("No monitored motors.");
                for (int i = 0; i < motors.length(); i++) {
                    JSONObject m = motors.optJSONObject(i);
                    lines.add(m.optString("name") + "  (" + m.optString("motorType") + ")");
                    lines.add(String.format(Locale.US, "  current avg %s  max %s A",
                            num(d(obj(m, "currentA"), "avg"), 2), num(d(obj(m, "currentA"), "max"), 2)));
                    lines.add(String.format(Locale.US, "  bemf avg %s  max %s V  vs batt %s",
                            num(d(obj(m, "bemfV"), "avg"), 2), num(d(obj(m, "bemfV"), "max"), 2),
                            num(d(obj(m, "bemfToBattery"), "max"), 2)));
                    lines.add(String.format(Locale.US, "  power max %s W  energy %s Wh  n=%d",
                            num(d(obj(m, "inputPowerW"), "max"), 1), num(d(m, "energyWh"), 4), m.optLong("samples")));
                }
                break;
            }
            default: {
                JSONArray devs = devices();
                if (devs.length() == 0) lines.add("No monitored devices.");
                for (int i = 0; i < devs.length(); i++) {
                    JSONObject dv = devs.optJSONObject(i);
                    lines.add(String.format(Locale.US, "%s%-12s %6d R %6d W %4s%% %6sms %4s%%",
                            i == Math.min(scroll, devs.length() - 1) ? ">" : " ",
                            trim(dv.optString("name"), 12), dv.optLong("reads"), dv.optLong("writes"),
                            num(d(dv, "cacheHitPct"), 0), num(d(dv, "avgReadLatencyUs") / 1000.0, 2),
                            num(d(dv, "ioSharePct"), 0)));
                }
                if (devs.length() > 0) lines.add(" name          reads   writes cache  avg rd  io share");
                break;
            }
        }
    }

    private void renderDevice(JSONObject dv, List<String> lines) {
        if (dv == null) {
            lines.add("No device.");
            return;
        }
        lines.add(dv.optString("type") + " / " + dv.optString("deviceClass")
                + (dv.optBoolean("cacheReads") ? "  cached" : ""));
        lines.add(dv.optString("connection"));
        lines.add(String.format(Locale.US, "Reads  %d (%d hw, %d cache = %s%%)", dv.optLong("reads"),
                dv.optLong("hardwareReads"), dv.optLong("cacheHits"), num(d(dv, "cacheHitPct"), 0)));
        lines.add(String.format(Locale.US, "Writes %d (%d redundant)", dv.optLong("writes"), dv.optLong("redundantWrites")));
        lines.add(String.format(Locale.US, "Rate   %s r/s  %s w/s (avg while active)",
                num(d(dv, "avgReadHz"), 1), num(d(dv, "avgWriteHz"), 1)));
        lines.add(String.format(Locale.US, "Read   avg %s us  max %s us",
                num(d(dv, "avgReadLatencyUs"), 0), num(d(dv, "maxReadLatencyUs"), 0)));
        lines.add(String.format(Locale.US, "Write  avg %s us  max %s us",
                num(d(dv, "avgWriteLatencyUs"), 0), num(d(dv, "maxWriteLatencyUs"), 0)));
        lines.add(String.format(Locale.US, "I/O    %s ms total (%s%% of all device I/O)",
                num(d(dv, "totalIoMs"), 1), num(d(dv, "ioSharePct"), 1)));
        lines.add(String.format(Locale.US, "Active reads %s-%s s  writes %s-%s s",
                num(d(dv, "firstReadS"), 1), num(d(dv, "lastReadS"), 1),
                num(d(dv, "firstWriteS"), 1), num(d(dv, "lastWriteS"), 1)));
        lines.add("-- operations: count  flagged  avg/max us  Hz");
        JSONArray ops = arr(dv, "ops");
        for (int i = 0; i < ops.length(); i++) {
            JSONObject o = ops.optJSONObject(i);
            long flagged = o.has("cacheHits") ? o.optLong("cacheHits") : o.optLong("redundant");
            lines.add(String.format(Locale.US, "%s %-20s %6d %5d %5s/%-6s %5s",
                    o.optString("kind"), trim(o.optString("op"), 20), o.optLong("count"), flagged,
                    num(d(o, "avgLatencyUs"), 0), num(d(o, "maxLatencyUs"), 0), num(d(o, "avgHz"), 1)));
        }
        lines.add("-- recent events: time s  op  latency us  flag");
        JSONArray recent = arr(dv, "recentEvents");
        for (int i = recent.length() - 1; i >= 0; i--) {
            JSONArray e = recent.optJSONArray(i);
            if (e == null) continue;
            lines.add(String.format(Locale.US, "%10.4f %-22s %8.1f %s", e.optDouble(0),
                    trim(e.optString(1), 22), e.optDouble(2), e.optBoolean(3) ? "*" : ""));
        }
    }

    // ------------------------------------------------------------------ JSON helpers

    private JSONArray devices() {
        return summary == null ? new JSONArray() : arr(summary, "devices");
    }

    private static JSONObject obj(JSONObject o, String key) {
        JSONObject v = o == null ? null : o.optJSONObject(key);
        return v == null ? new JSONObject() : v;
    }

    private static JSONArray arr(JSONObject o, String key) {
        JSONArray v = o == null ? null : o.optJSONArray(key);
        return v == null ? new JSONArray() : v;
    }

    private static double d(JSONObject o, String key) {
        return o == null ? Double.NaN : o.optDouble(key, Double.NaN);
    }

    private static String num(double v, int decimals) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "-";
        return String.format(Locale.US, "%." + decimals + "f", v);
    }

    private static String trim(String s, int max) {
        if (s == null) return "";
        return s.length() <= max ? s : s.substring(0, max - 1) + "~";
    }

    private static String shortStart(String start) {
        // "2026-10-23 13:05:22" -> "10-23 13:05"
        return start != null && start.length() >= 16 ? start.substring(5, 16) : String.valueOf(start);
    }

    private static String mb(long bytes) {
        return String.format(Locale.US, "%.1f", bytes / 1048576.0);
    }
}
