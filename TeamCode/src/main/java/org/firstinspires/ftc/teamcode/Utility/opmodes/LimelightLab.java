package org.firstinspires.ftc.teamcode.Utility.opmodes;

import android.content.res.AssetManager;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.Systems.Vision.LimelightBridge;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@TeleOp(name = "Limelight Lab", group = "Diagnostics")
public class LimelightLab extends LinearOpMode {

    private static final double HOLD_SECONDS = 1.0;
    private static final double STATUS_PERIOD_SECONDS = 1.0;
    private static final int PIPELINES = 10;
    private static final Pattern ASSET = Pattern.compile("p(\\d)_.+\\.(py|json)");

    @Override
    public void runOpMode() {

        Limelight3A limelight = null;
        try {
            limelight = hardwareMap.get(Limelight3A.class, DeviceConstants.limelightCameraDeviceName);
        }
        catch (RuntimeException ignored) {
        }
        LimelightBridge bridge = new LimelightBridge(address(limelight), VisionConstants.LIMELIGHT_BRIDGE_FIRST_PORT, VisionConstants.LIMELIGHT_BRIDGE_FIRST_PORT, VisionConstants.LIMELIGHT_BRIDGE_PORT_COUNT, VisionConstants.LIMELIGHT_BRIDGE_MAX_CONNECTIONS);
        bridge.start();
        AssetManager assets = hardwareMap.appContext.getAssets();
        List<String> files = assetFiles(assets);
        int pipeline = VisionConstants.LIMELIGHT_LAB_START_PIPELINE;
        if (limelight != null) {
            limelight.setPollRateHz(VisionConstants.LIMELIGHT_POLL_RATE_HZ);
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        }
        ElapsedTime held = new ElapsedTime();
        ElapsedTime statusAge = new ElapsedTime();
        boolean holding = false, lastUp = false, lastDown = false, lastX = false;
        int snapshots = 0;
        String report = "", status = "";
        try {
            while (!isStopRequested()) {
                HardwareMonitor.get().update();
                telemetry.addLine("LIMELIGHT LAB (moves nothing)");
                telemetry.addLine("Laptop on robot Wi-Fi: http://192.168.43.1:5801");
                telemetry.addLine("REST :5807, stream :5800, CLI: python tools/limelight.py");
                telemetry.addLine(String.format(Locale.US, "Bridge -> %s: %d/%d ports, %d open, %.1f MB", bridge.host(), bridge.openPorts(), VisionConstants.LIMELIGHT_BRIDGE_PORT_COUNT, bridge.connections(), bridge.bytes() / 1048576.0));
                if (!bridge.lastError().isEmpty()) telemetry.addLine("Bridge: " + bridge.lastError());
                if (limelight == null) {
                    telemetry.addLine("No Limelight called '" + DeviceConstants.limelightCameraDeviceName + "' in the config.");
                }
                else {
                    boolean up = gamepad1.dpad_up;
                    boolean down = gamepad1.dpad_down;
                    if (up && !lastUp) pipeline = (pipeline + 1) % PIPELINES;
                    if (down && !lastDown) pipeline = (pipeline + PIPELINES - 1) % PIPELINES;
                    if (up && !lastUp || down && !lastDown) limelight.pipelineSwitch(pipeline);
                    lastUp = up;
                    lastDown = down;
                    if (gamepad1.x && !lastX) {
                        snapshots++;
                        boolean ok = limelight.captureSnapshot("lab_" + snapshots);
                        report = "Snapshot lab_" + snapshots + (ok ? " saved" : " FAILED");
                    }
                    lastX = gamepad1.x;
                    boolean both = gamepad1.a && gamepad1.b;
                    if (both && !holding) held.reset();
                    holding = both;
                    if (holding && held.seconds() >= HOLD_SECONDS) {
                        report = upload(limelight, assets, files, pipeline);
                        holding = false;
                        held.reset();
                        while (gamepad1.a && gamepad1.b && !isStopRequested()) idle();
                    }
                    if (statusAge.seconds() >= STATUS_PERIOD_SECONDS) {
                        status = status(limelight.getStatus());
                        statusAge.reset();
                    }
                    telemetry.addLine(String.format(Locale.US, "Limelight %s, connected %b", limelight.getConnectionInfo(), limelight.isConnected()));
                    telemetry.addLine(status);
                    showResult(limelight.getLatestResult(), pipeline);
                }
                telemetry.addLine("");
                telemetry.addLine("Assets: " + (files.isEmpty() ? "none" : files.toString()));
                if (holding) telemetry.addLine(String.format(Locale.US, "Uploading in %.1f s ...", Math.max(0, HOLD_SECONDS - held.seconds())));
                if (!report.isEmpty()) telemetry.addLine(report);
                telemetry.addLine("dpad up/down pipeline, X snapshot, hold A+B upload assets (overwrites those pipelines)");
                telemetry.update();
                sleep(50);
            }
        }
        finally {
            bridge.stop();
            if (limelight != null) limelight.stop();
        }
    }

    private void showResult(LLResult result, int pipeline) {

        if (result == null) {
            telemetry.addLine("No result yet");
            return;
        }
        String switching = result.getPipelineIndex() == pipeline ? "" : "  (switching to " + pipeline + ")";
        telemetry.addLine(String.format(Locale.US, "Pipeline %d %s%s", result.getPipelineIndex(), result.getPipelineType(), switching));
        telemetry.addLine(String.format(Locale.US, "valid %b  tx %.2f  ty %.2f  ta %.3f", result.isValid(), result.getTx(), result.getTy(), result.getTa()));
        telemetry.addLine(String.format(Locale.US, "stale %d ms, capture %.1f + targeting %.1f ms", result.getStaleness(), result.getCaptureLatency(), result.getTargetingLatency()));
        telemetry.addLine(String.format(Locale.US, "color %d, tags %d, detector %d", count(result.getColorResults()), count(result.getFiducialResults()), count(result.getDetectorResults())));
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections != null && !detections.isEmpty()) telemetry.addLine(String.format(Locale.US, "best: %s %.2f", detections.get(0).getClassName(), detections.get(0).getConfidence()));
        double[] python = result.getPythonOutput();
        if (python != null && python.length > 0) telemetry.addLine("python " + Arrays.toString(Arrays.copyOf(python, Math.min(python.length, 8))));
    }

    private static int count(List<?> list) {
        return list == null ? 0 : list.size();
    }

    private static String status(LLStatus status) {

        if (status == null) return "No status";
        return String.format(Locale.US, "%s: pipeline %d %s, %.0f fps, %.0f C, cpu %.0f%%", status.getName(), status.getPipelineIndex(), status.getPipelineType(), status.getFps(), status.getTemp(), status.getCpu());
    }

    private static String upload(Limelight3A limelight, AssetManager assets, List<String> files, int pipeline) {

        if (files.isEmpty()) return "Nothing in assets/" + VisionConstants.LIMELIGHT_ASSET_DIRECTORY;
        StringBuilder report = new StringBuilder("Upload:");
        for (int pass = 0; pass < 2; pass++) {
            for (String file : files) {
                boolean json = file.endsWith(".json");
                if (json != (pass == 0)) continue;
                Matcher m = ASSET.matcher(file);
                if (!m.matches()) continue;
                int index = Integer.parseInt(m.group(1));
                boolean ok;
                try {
                    String text = read(assets, VisionConstants.LIMELIGHT_ASSET_DIRECTORY + "/" + file);
                    ok = json ? limelight.uploadPipeline(text, index) : limelight.uploadPython(text, index);
                }
                catch (IOException e) {
                    ok = false;
                }
                report.append(' ').append(file).append(ok ? " ok" : " FAILED").append(',');
            }
        }
        limelight.pipelineSwitch(pipeline);
        limelight.reloadPipeline();
        return report.toString();
    }

    private static List<String> assetFiles(AssetManager assets) {

        List<String> files = new ArrayList<>();
        try {
            String[] names = assets.list(VisionConstants.LIMELIGHT_ASSET_DIRECTORY);
            if (names != null) {
                for (String name : names) {
                    if (ASSET.matcher(name).matches()) files.add(name);
                }
            }
        }
        catch (IOException ignored) {
        }
        return files;
    }

    private static String read(AssetManager assets, String path) throws IOException {

        try (InputStream stream = assets.open(path)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            int n;
            while ((n = stream.read(buffer)) > 0) out.write(buffer, 0, n);
            return new String(out.toByteArray(), "UTF-8");
        }
    }

    //the SDK doesn't expose the Limelight's address
    private static String address(Limelight3A limelight) {

        if (limelight != null) {
            try {
                Field field = Limelight3A.class.getDeclaredField("inetAddress");
                field.setAccessible(true);
                Object value = field.get(limelight);
                if (value instanceof InetAddress) return ((InetAddress) value).getHostAddress();
            }
            catch (Exception ignored) {
            }
        }
        return VisionConstants.LIMELIGHT_FALLBACK_ADDRESS;
    }
}
