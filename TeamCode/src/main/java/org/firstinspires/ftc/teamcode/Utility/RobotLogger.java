package org.firstinspires.ftc.teamcode.Utility;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class RobotLogger {

    public static boolean deleteLogFile(HardwareMap hardwareMap, String fileName) {

        File file = new File(hardwareMap.appContext.getFilesDir(), fileName);

        if (file.exists()) {
            return file.delete();
        }

        return false;
    }

    private static final SimpleDateFormat timeFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US);

    public static void logToFile(HardwareMap hardwareMap, String tag, String msg, String fileName) {

        try {
            File file = new File(hardwareMap.appContext.getFilesDir(), fileName);
            FileWriter writer = new FileWriter(file, true);

            String timestamp = timeFormat.format(new Date());

            writer.write(timestamp + " | " + tag + " | " + msg + "\n");
            writer.close();

        } catch (IOException ignored) {}
    }

    public static void debug(String tag, String msg) {
        Log.d(tag, msg);
    }

    public static void info(String tag, String msg) {
        Log.i(tag, msg);
    }

    public static void warning(String tag, String msg) {
        Log.w(tag, msg);
    }

    public static void error(String tag, String msg) {
        Log.e(tag, msg);
    }
}