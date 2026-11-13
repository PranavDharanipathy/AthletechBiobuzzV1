package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.Utility.CsvLogger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@Config
@TeleOp(name = "Turret Autotuner", group = "tuning")
public class TurretAutotuner extends TeleOpBaseOpMode {

    public static int ROUNDS = TurretAutotuneEngine.ROUNDS;

    private static final String[] COLUMNS = {"t", "stage", "target", "position", "volts", "available_volts", "p", "i", "d", "f", "s", "v"};

    private TurretAutotuneEngine engine;
    private CsvLogger logger;
    private final double[] row = new double[COLUMNS.length];
    private File folder;
    private String lastStage = "";
    private int stageNumber = 0;
    private boolean saved = false, stopped = false;
    private double startTime;
    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeComponents(Alliance.BLUE);
        applyComponentTraits();
        recorder.channel("autotune", "target", "deg", "what the engine asks for", () -> engine != null ? engine.getTarget() / ShooterConstants.TURRET_TICKS_PER_DEGREE : Double.NaN);
        recorder.channel("autotune", "volts", "V", "", () -> engine != null ? engine.getVolts() : Double.NaN);
        recorder.watch("autotune", "stage", () -> engine != null ? engine.getStage() : null);
        record(null, null, null, null);
        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        telemetry.addLine("TURRET AUTOTUNER");
        telemetry.addLine("Turret straight ahead at INIT (that's 0). Robot still, hands and wires clear: it swings up to ~90 deg each way.");
        telemetry.addLine("Measures friction and speed per volt, tunes the gains with step moves, then runs a match course (ramps, wobble, nudges, holds) to tune out shake. About " + (5 + 5 * ROUNDS) + " minutes.");
        telemetry.addLine("B during the match course keeps what's tuned so far; B before it throws the run away.");
        telemetry.addData("Rounds (dashboard ROUNDS)", ROUNDS);
        telemetry.update();
    }

    @Override
    public void start() {

        turret.setVoltageCompensation(TurretConstants.TURRET_VOLTAGE_COMPENSATION);
        double[] knobs = {TurretConstants.TURRET_KP_FAR, TurretConstants.TURRET_KP_CLOSE, TurretConstants.TURRET_KI_FAR, TurretConstants.TURRET_KI_CLOSE, TurretConstants.TURRET_KD_FAR, TurretConstants.TURRET_KD_CLOSE, TurretConstants.TURRET_K_HOLD_VOLTS, TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES, TurretConstants.TURRET_MAX_I, TurretConstants.TURRET_STRIBECK_DEGREES_PER_SECOND};
        engine = new TurretAutotuneEngine(ShooterConstants.TURRET_TICKS_PER_DEGREE, knobs, TurretConstants.TURRET_GAIN, TurretConstants.TURRET_KINETIC_FRICTION, TurretConstants.TURRET_BREAKAWAY_FRICTION);
        engine.rounds = Math.max(1, ROUNDS);
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        folder = new File(new File(AppUtil.ROBOT_DATA_DIR, "TurretAutotune"), stamp);
        logger = new CsvLogger(new File(folder, "samples.csv"), COLUMNS);
        startTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();
        controller1.getInformation();
        //B after the step moves keeps what's tuned; before that, nothing is used
        if (controller1.bHasJustBeenPressed && engine != null && !engine.keepEarly()) stopped = true;

        turret.readSensors();
        double now = System.nanoTime() * 1e-9 - startTime;
        if (!stopped) engine.update(now, turret.getPosition() - turret.startPosition);
        double volts = stopped ? 0 : engine.getVolts();
        turret.setVolts(volts, volts);

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
        flywheel.setVelocity(0, true);
        flywheel.update();

        if (!engine.getStage().equals(lastStage)) {
            lastStage = engine.getStage();
            stageNumber++;
        }
        log(now, volts);
        if ((engine.isDone() || engine.isFaulted() || stopped) && !saved) save();
        show();
    }

    @Override
    public void stop() {

        if (engine != null && !saved) save();
        if (logger != null) logger.close();
        turret.setVolts(0, 0);
        super.stop();
    }

    private void log(double now, double volts) {

        int k = 0;
        row[k++] = now;
        row[k++] = stageNumber;
        row[k++] = engine.getTarget();
        row[k++] = turret.getPosition() - turret.startPosition;
        row[k++] = volts;
        row[k++] = turret.getCompensationVolts();
        row[k++] = engine.getController().p;
        row[k++] = engine.getController().i;
        row[k++] = engine.getController().d;
        row[k++] = engine.getController().f;
        row[k++] = engine.getController().s;
        row[k++] = engine.getController().v;
        logger.row(row);
    }

    private void show() {

        telemetry.addData("Stage", engine.getStage());
        telemetry.addLine(stopped ? "STOPPED (B): not used" : engine.getStatus());
        telemetry.addData("Turret (deg)", "%.1f -> %.1f", (turret.getPosition() - turret.startPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE, engine.getTarget() / ShooterConstants.TURRET_TICKS_PER_DEGREE);
        List<String> history = engine.getHistory();
        for (int k = Math.max(0, history.size() - 8); k < history.size(); k++) telemetry.addLine(history.get(k));
        if (engine.isDone()) {
            telemetry.addLine("DONE. Saved to " + folder.getPath());
            telemetry.addLine("Plug in and build: it goes into TunedConstants.TURRET_* by itself.");
            for (Map.Entry<String, Double> e : engine.getResults().entrySet()) telemetry.addData(e.getKey(), "%.4g", e.getValue());
        }
        if (engine.isFaulted()) telemetry.addLine("FAULT: " + engine.getStatus());
        telemetry.update();
    }

    private void save() {

        saved = true;
        StringBuilder b = new StringBuilder("key,value\n");
        b.append("status,").append(engine.isDone() ? "done" : engine.isFaulted() ? "fault" : "stopped").append('\n');
        b.append("message,").append(engine.getStatus().replace(',', ';')).append('\n');
        b.append("ticks_per_degree,").append(ShooterConstants.TURRET_TICKS_PER_DEGREE).append('\n');
        b.append("rounds,").append(engine.rounds).append('\n');
        if (engine.isDone()) {
            for (Map.Entry<String, Double> e : engine.getResults().entrySet()) b.append(e.getKey()).append(',').append(e.getValue()).append('\n');
        }
        for (String h : engine.getHistory()) b.append("history,").append(h.replace(',', ';')).append('\n');
        folder.mkdirs();
        File file = new File(folder, "result.csv");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(file), Charset.forName("UTF-8"))) {
            w.write(b.toString());
        } catch (IOException e) {
            telemetry.log().add("couldn't write " + file);
        }
    }
}
