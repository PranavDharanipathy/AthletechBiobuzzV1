package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import android.content.Context;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

/**
 * Serves the HardwareMonitor dashboard from the Robot Controller's built-in web server - the
 * same server that hosts OnBotJava and Blocks. Connect a laptop or phone to the robot's Wi-Fi
 * and open:
 *
 * <pre>
 *   Control Hub:              http://192.168.43.1:8080/hwmonitor
 *   Phone Robot Controller:   http://192.168.49.1:8080/hwmonitor
 * </pre>
 *
 * The page itself is {@code TeamCode/src/main/assets/hwmonitor/index.html}. The same file can be
 * opened straight from disk on a laptop to browse a folder copied off the robot with
 * {@code adb pull /sdcard/FIRST/data/HardwareMonitor}.
 *
 * <h3>JSON / CSV API</h3>
 * <pre>
 * GET  /hwmonitor/api/sessions                          list of sessions with key metrics
 * GET  /hwmonitor/api/file?session=ID&amp;name=FILE[&amp;offset=N][&amp;download=1]
 *                                                        summary.json, system.csv, motors.csv,
 *                                                        devices.csv or events.csv; "offset"
 *                                                        skips bytes (header X-Next-Offset
 *                                                        returns where to continue from)
 * GET  /hwmonitor/api/live                              summary of the recording session, or 204
 * POST /hwmonitor/api/delete?session=ID                 delete one stored session
 * POST /hwmonitor/api/clear                             delete every stored session
 * </pre>
 */
public final class MonitorWebServer {

    public static final String PATH = "/hwmonitor";
    static final String ASSET = "hwmonitor/index.html";
    static final String[] SESSION_FILES = {"summary.json", "system.csv", "motors.csv", "devices.csv", "events.csv"};

    private static final String MIME_JSON = "application/json";
    private static final String MIME_CSV = "text/csv";
    private static final String TAG = HardwareMonitor.TAG;

    private MonitorWebServer() {
    }

    /** Found and called by the SDK when the web server starts (like FTC Dashboard's pages). */
    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        try {
            register(manager, context.getAssets());
        } catch (Throwable t) {
            // Never let the dashboard break the Robot Controller.
            RobotLog.ee(TAG, t, "HardwareMonitor: could not register the web dashboard");
        }
    }

    static void register(WebHandlerManager manager, final AssetManager assets) {
        WebHandler page = new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws IOException {
                if (session.getMethod() != NanoHTTPD.Method.GET) return status(NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED);
                NanoHTTPD.Response r = NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        "text/html; charset=utf-8", assets.open(ASSET));
                r.addHeader("Cache-Control", "no-cache");
                return r;
            }
        };
        manager.register(PATH, page);
        manager.register(PATH + "/", page);
        manager.register(PATH + "/index.html", page);

        manager.register(PATH + "/api/sessions", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                try {
                    return json(sessionsJson());
                } catch (Exception e) {
                    return error(e);
                }
            }
        });

        manager.register(PATH + "/api/file", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                try {
                    return file(query(session));
                } catch (Exception e) {
                    return error(e);
                }
            }
        });

        manager.register(PATH + "/api/live", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                try {
                    String live = HardwareMonitor.get().liveSummaryJson();
                    if (live == null) return status(NanoHTTPD.Response.Status.NO_CONTENT);
                    return json(live);
                } catch (Exception e) {
                    return error(e);
                }
            }
        });

        manager.register(PATH + "/api/delete", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                if (session.getMethod() != NanoHTTPD.Method.POST) return status(NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED);
                try {
                    String id = query(session).get("session");
                    if (!validId(id)) return text(NanoHTTPD.Response.Status.BAD_REQUEST, "bad session id");
                    if (id.equals(HardwareMonitor.get().getSessionId())) {
                        return text(NanoHTTPD.Response.Status.BAD_REQUEST, "that session is still recording");
                    }
                    boolean ok = MonitorStore.deleteSession(id);
                    return json("{\"deleted\":" + ok + "}");
                } catch (Exception e) {
                    return error(e);
                }
            }
        });

        manager.register(PATH + "/api/clear", new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                if (session.getMethod() != NanoHTTPD.Method.POST) return status(NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED);
                try {
                    return json("{\"deleted\":" + HardwareMonitor.clearDatabase() + "}");
                } catch (Exception e) {
                    return error(e);
                }
            }
        });

        RobotLog.ii(TAG, "HardwareMonitor dashboard available at %s", PATH);
    }

    // ------------------------------------------------------------------ handlers

    static String sessionsJson() throws JSONException {
        String recording = HardwareMonitor.get().getSessionId();
        List<MonitorStore.SessionInfo> list = MonitorStore.listSessions();
        JSONObject root = new JSONObject();
        root.put("recording", recording == null ? JSONObject.NULL : recording);
        root.put("lifecycleHook", HardwareMonitor.get().isLifecycleHookActive());
        root.put("path", MonitorStore.rootDir().getPath());
        long total = 0;
        JSONArray arr = new JSONArray();
        for (MonitorStore.SessionInfo s : list) {
            JSONObject o = new JSONObject();
            o.put("id", s.id);
            o.put("opMode", s.opMode);
            o.put("start", s.start);
            num(o, "durationS", s.durationS);
            o.put("loops", s.loops);
            num(o, "avgLoopMs", s.avgLoopMs);
            num(o, "p95LoopMs", s.p95LoopMs);
            num(o, "minBatteryV", s.minBatteryV);
            num(o, "energyWh", s.energyWh);
            o.put("devices", s.devices);
            o.put("complete", s.complete);
            o.put("recording", s.id.equals(recording));
            o.put("sizeBytes", s.sizeBytes);
            JSONObject files = new JSONObject();
            for (String name : SESSION_FILES) {
                File f = new File(s.dir, name);
                if (f.isFile()) files.put(name, f.length());
            }
            o.put("files", files);
            arr.put(o);
            total += s.sizeBytes;
        }
        root.put("totalBytes", total);
        root.put("sessions", arr);
        return root.toString();
    }

    static NanoHTTPD.Response file(Map<String, String> q) throws IOException {
        String id = q.get("session");
        String name = q.get("name");
        if (!validId(id) || !isSessionFile(name)) {
            return text(NanoHTTPD.Response.Status.BAD_REQUEST, "bad session or file name");
        }
        File f = new File(new File(MonitorStore.sessionsDir(), id), name);
        if (!f.isFile()) return text(NanoHTTPD.Response.Status.NOT_FOUND, "no such file");

        long length = f.length();
        long offset = 0;
        try {
            if (q.get("offset") != null) offset = Long.parseLong(q.get("offset"));
        } catch (NumberFormatException ignored) {
            // treat as 0
        }
        offset = Math.max(0, Math.min(offset, length));

        InputStream in = new FileInputStream(f);
        long skipped = 0;
        while (skipped < offset) {
            long n = in.skip(offset - skipped);
            if (n <= 0) break;
            skipped += n;
        }
        String mime = name.endsWith(".json") ? MIME_JSON : MIME_CSV;
        NanoHTTPD.Response r = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                mime + "; charset=utf-8", in, length - offset);
        r.addHeader("X-Next-Offset", Long.toString(length));
        r.addHeader("Cache-Control", "no-store");
        if ("1".equals(q.get("download"))) {
            r.addHeader("Content-Disposition", "attachment; filename=\"" + id + "_" + name + "\"");
        }
        return r;
    }

    // ------------------------------------------------------------------ helpers

    static boolean validId(String id) {
        if (id == null || id.isEmpty() || id.length() > 120 || id.startsWith(".")) return false;
        for (int i = 0; i < id.length(); i++) {
            char c = id.charAt(i);
            if (!(Character.isLetterOrDigit(c) || c == '-' || c == '_' || c == '.')) return false;
        }
        return true;
    }

    static boolean isSessionFile(String name) {
        if (name == null) return false;
        for (String allowed : SESSION_FILES) if (allowed.equals(name)) return true;
        return false;
    }

    static Map<String, String> query(NanoHTTPD.IHTTPSession session) throws UnsupportedEncodingException {
        Map<String, String> out = new HashMap<>();
        String qs = session.getQueryParameterString();
        if (qs == null || qs.isEmpty()) return out;
        for (String pair : qs.split("&")) {
            if (pair.isEmpty()) continue;
            int eq = pair.indexOf('=');
            String k = URLDecoder.decode(eq < 0 ? pair : pair.substring(0, eq), "UTF-8");
            String v = eq < 0 ? "" : URLDecoder.decode(pair.substring(eq + 1), "UTF-8");
            out.put(k, v);
        }
        return out;
    }

    private static void num(JSONObject o, String key, double v) throws JSONException {
        if (Double.isNaN(v) || Double.isInfinite(v)) o.put(key, JSONObject.NULL);
        else o.put(key, v);
    }

    private static NanoHTTPD.Response json(String body) {
        NanoHTTPD.Response r = NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                MIME_JSON + "; charset=utf-8", body);
        r.addHeader("Cache-Control", "no-store");
        return r;
    }

    private static NanoHTTPD.Response text(NanoHTTPD.Response.Status status, String body) {
        return NanoHTTPD.newFixedLengthResponse(status, NanoHTTPD.MIME_PLAINTEXT, body);
    }

    private static NanoHTTPD.Response status(NanoHTTPD.Response.Status status) {
        return NanoHTTPD.newFixedLengthResponse(status, NanoHTTPD.MIME_PLAINTEXT, "");
    }

    private static NanoHTTPD.Response error(Exception e) {
        RobotLog.ee(TAG, e, "HardwareMonitor web request failed");
        return text(NanoHTTPD.Response.Status.INTERNAL_ERROR, String.valueOf(e));
    }
}
