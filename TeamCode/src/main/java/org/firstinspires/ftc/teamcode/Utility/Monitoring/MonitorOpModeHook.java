package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

/**
 * Connects {@link HardwareMonitor} to the OpMode lifecycle so sessions start when an OpMode is
 * initialised and are saved when it stops - even if your code never calls
 * {@link HardwareMonitor#stop()}, throws, or is force-stopped from the Driver Station.
 *
 * <p>The SDK finds the {@link OnCreateEventLoop} method below by scanning the app, the same way
 * FTC Dashboard attaches itself. Nothing needs to call it.
 */
public final class MonitorOpModeHook implements OpModeManagerNotifier.Notifications {

    private static final MonitorOpModeHook INSTANCE = new MonitorOpModeHook();
    private static OpModeManagerImpl attachedManager;

    private MonitorOpModeHook() {
    }

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        try {
            OpModeManagerImpl manager = eventLoop.getOpModeManager();
            synchronized (MonitorOpModeHook.class) {
                if (attachedManager != null && attachedManager != manager) {
                    attachedManager.unregisterListener(INSTANCE);
                }
                if (manager != null) {
                    manager.registerListener(INSTANCE);
                    attachedManager = manager;
                }
            }
            HardwareMonitor.get().setHooked(manager != null);
        } catch (Throwable t) {
            // Never let monitoring break robot start-up.
            RobotLog.ee(HardwareMonitor.TAG, t, "HardwareMonitor: could not attach to the OpMode manager");
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        try {
            HardwareMonitor.get().onOpModePreInit(opMode);
        } catch (Throwable t) {
            RobotLog.ee(HardwareMonitor.TAG, t, "HardwareMonitor: onOpModePreInit failed");
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        try {
            HardwareMonitor.get().onOpModePreStart(opMode);
        } catch (Throwable t) {
            RobotLog.ee(HardwareMonitor.TAG, t, "HardwareMonitor: onOpModePreStart failed");
        }
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        try {
            HardwareMonitor.get().onOpModePostStop(opMode);
        } catch (Throwable t) {
            RobotLog.ee(HardwareMonitor.TAG, t, "HardwareMonitor: onOpModePostStop failed");
        }
    }
}
