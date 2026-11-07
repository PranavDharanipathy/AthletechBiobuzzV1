package org.firstinspires.ftc.teamcode.Tuners;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.DriveController;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredVoltageSensor;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Map;

/// Measures the drivetrain and tunes the pose hold (DriveAutotuneEngine). The robot drives itself: forward and back, strafes,
/// spins, then shoves itself while holding. Needs about 3 ft clear every way. Any stick or B stops it. The result goes to
/// /sdcard/FIRST/data/DriveAutotune/ and the next build puts it in TunedConstants
@TeleOp(name = "Drivetrain Autotuner", group = "tuning")
public class DrivetrainAutotuner extends TeleOpBaseOpMode {

    private DriveAutotuneEngine engine;
    private final VoltageCompensator voltage = new VoltageCompensator(0.05, 0.1, 12);
    private MonitoredVoltageSensor battery;
    private final double[] wheels = new double[4], idle = new double[3];
    private boolean started, saved;
    private File folder;

    @Override
    public void init() {

        initializeComponents(Alliance.RED);
        applyComponentTraits();
        battery = new MonitoredVoltageSensor(flywheel.getVoltageSensor(), "driveAutotuneBattery", false);
        engine = new DriveAutotuneEngine();
        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        telemetry.addLine("DRIVETRAIN AUTOTUNER");
        telemetry.addLine("The robot drives itself for about a minute: forward/back, strafes, spins, then shoves itself.");
        telemetry.addLine("Clear about 3 ft every way (it stops if it gets 6 ft from where it started).");
        telemetry.addLine("After START press A. Any stick or B stops it at once.");
        telemetry.update();
    }

    @Override
    public void start() {
        folder = new File(new File(AppUtil.ROBOT_DATA_DIR, "DriveAutotune"), new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()));
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();
        controller1.getInformation();
        follower.update();
        double now = Clock.seconds();
        if (voltage.shouldSample(now)) voltage.addSample(battery.getVoltage(), now);
        if (!started && controller1.aHasJustBeenPressed) started = true;
        boolean sticks = Math.abs(controller1.left_stick_x()) > 0.3 || Math.abs(controller1.left_stick_y()) > 0.3 || Math.abs(controller1.right_stick_x()) > 0.3;
        if (started && !engine.isDone() && !engine.isFaulted() && (sticks || controller1.b())) engine.stop("stopped by the driver");
        if (started) {
            Pose pose = follower.pose();
            Velocity velocity = follower.velocity();
            engine.update(now, pose.x(), pose.y(), pose.heading(), velocity.vx, velocity.vy, velocity.omega, voltage.volts());
        }
        double[] u = started ? engine.getPower() : idle;
        DriveController.wheels(u[0], u[1], u[2], wheels);
        left_front.setPower(wheels[0]);
        right_front.setPower(wheels[1]);
        left_back.setPower(wheels[2]);
        right_back.setPower(wheels[3]);
        if ((engine.isDone() || engine.isFaulted()) && !saved) save();

        telemetry.addLine("DRIVETRAIN AUTOTUNER");
        telemetry.addLine(started ? engine.getStatus() : ">>> PRESS A TO START <<<");
        if (engine.isDone()) {
            telemetry.addLine("DONE. Saved to " + folder.getPath());
            telemetry.addLine("Plug in and build: it goes into TunedConstants by itself.");
            for (Map.Entry<String, String> e : engine.getResults().entrySet()) if (!e.getKey().contains("_trial_")) telemetry.addData(e.getKey(), e.getValue());
        }
        telemetry.update();
    }

    @Override
    public void stop() {

        super.stop();
        if (engine != null && started && !saved) save();
    }

    private void save() {

        saved = true;
        StringBuilder b = new StringBuilder("key,value\n");
        b.append("status,").append(engine.isDone() ? "done" : engine.isFaulted() ? "fault" : "stopped").append('\n');
        b.append("detail,").append(engine.getStatus().replace(',', ' ')).append('\n');
        for (Map.Entry<String, String> e : engine.getResults().entrySet()) b.append(e.getKey()).append(',').append(e.getValue().replace(',', ' ')).append('\n');
        if (!folder.isDirectory() && !folder.mkdirs()) {
            telemetry.log().add("couldn't make " + folder);
            return;
        }
        File file = new File(folder, "result.csv");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
            w.write(b.toString());
        } catch (IOException e) {
            telemetry.log().add("couldn't write " + file);
        }
    }
}
