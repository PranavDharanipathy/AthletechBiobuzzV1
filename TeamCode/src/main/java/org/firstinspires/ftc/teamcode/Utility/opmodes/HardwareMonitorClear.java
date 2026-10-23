package org.firstinspires.ftc.teamcode.Utility.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.MonitorStore;

import java.util.Locale;

/**
 * Deletes the whole HardwareMonitor database from the Driver Station. Works during INIT.
 * Hold A + B on gamepad 1 for one second to confirm, so it cannot happen by accident.
 *
 * <p>From code, the same command is {@link HardwareMonitor#clearDatabase()}.
 */
@TeleOp(name = "Hardware Monitor: Clear Database", group = "Diagnostics")
public class HardwareMonitorClear extends LinearOpMode {

    private static final double HOLD_SECONDS = 1.0;

    @Override
    public void runOpMode() {
        ElapsedTime held = new ElapsedTime();
        boolean holding = false;
        int deleted = -1;

        int sessions = MonitorStore.listSessions().size();
        long bytes = MonitorStore.totalSizeBytes();

        while (!isStopRequested()) {
            telemetry.addLine("HARDWARE MONITOR DATABASE");
            telemetry.addLine(MonitorStore.rootDir().getPath());
            if (deleted >= 0) {
                telemetry.addLine(String.format(Locale.US, "Deleted %d session(s).", deleted));
                telemetry.addLine("You can stop this OpMode.");
            } else {
                telemetry.addLine(String.format(Locale.US, "%d session(s), %.1f MB", sessions, bytes / 1048576.0));
                telemetry.addLine("");
                if (sessions == 0) {
                    telemetry.addLine("Nothing to delete.");
                } else {
                    telemetry.addLine("Hold A + B for 1 second to delete EVERYTHING.");
                    boolean both = gamepad1.a && gamepad1.b;
                    if (both && !holding) {
                        holding = true;
                        held.reset();
                    } else if (!both) {
                        holding = false;
                    }
                    if (holding) {
                        double remaining = Math.max(0, HOLD_SECONDS - held.seconds());
                        telemetry.addLine(String.format(Locale.US, "Deleting in %.1f s ...", remaining));
                        if (remaining == 0) deleted = HardwareMonitor.clearDatabase();
                    }
                }
            }
            telemetry.update();
            sleep(50);
        }
    }
}
