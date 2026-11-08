package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.BlazeConstants;
import org.firstinspires.ftc.teamcode.Constants.BlockerConstants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.TemporalLog.TemporalLog;

import java.io.File;

@TeleOp(name = "Temporal Log Red", group = "A_Match")
public class TemporalLogRedTeleOp extends RedTeleOp {

    private File dir;

    @Override
    public void init() {

        //open before hardware so INIT's writes are in
        dir = TemporalLog.open("Temporal Log Red");
        TemporalLog.mark("phase.init", 0);
        super.init();
        TemporalLog.watch(this, "teleop", FieldConstants.class, GeneralConstants.class, DrivetrainConstants.class, TunedConstants.class, FlywheelConstants.class, ShooterConstants.class, TurretConstants.class, IntakeConstants.class, BlockerConstants.class, BlazeConstants.class, BlazeSession.class);
        TemporalLog.mark("phase.init_done", TemporalLog.variableCount());
        telemetry.addData("Temporal log", () -> dir == null ? "FAILED to open (see logcat)" : dir.getPath());
        telemetry.addData("Logged", () -> TemporalLog.records + " records, " + TemporalLog.variableCount() + " variables, " + TemporalLog.keyCount() + " keys, " + TemporalLog.dropped + " dropped");
    }

    @Override
    public void init_loop() {

        TemporalLog.beginLoop();
        TemporalLog.endLoop();
    }

    @Override
    public void start() {

        TemporalLog.mark("phase.start", 0);
        super.start();
    }

    @Override
    public void loop() {

        TemporalLog.beginLoop();
        super.loop();
        TemporalLog.endLoop();
    }

    @Override
    public void stop() {

        TemporalLog.mark("phase.stop", 0);
        super.stop();
        TemporalLog.close();
    }
}
