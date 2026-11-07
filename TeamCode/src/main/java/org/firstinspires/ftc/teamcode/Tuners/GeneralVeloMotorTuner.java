package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;

@Config
@TeleOp (group = "tuning")
public class GeneralVeloMotorTuner extends OpMode {

    public static long LOOP_TIME = 60;

    public static String motorName = "";

    private GeneralVeloMotor motor;

    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double VELOCITY = 0;

    public static double KP = 0;
    public static double KD = 0;
    public static double KFF = 0;

    private Telemetry telemetry;

    @Override
    public void init() {

        BlazeSession.begin(hardwareMap);
        motor = new GeneralVeloMotor(hardwareMap, motorName);
        BlazeSession.engage(hardwareMap, null);

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {

        BlazeSession.tick();
        motor.setDirection(DIRECTION);

        motor.setVelocityPDFCoefficients(KP, KD, KFF);

        motor.setVelocity(VELOCITY);

        motor.update();

        telemetry.addData("velocity", motor.getVelocity());
        telemetry.addData("power", motor.getPower());
        telemetry.addData("p", motor.p);
        telemetry.addData("d", motor.d);
        telemetry.addData("ff", motor.ff);
        telemetry.update();

        sleep(LOOP_TIME);
    }

    @Override
    public void stop() {
        BlazeSession.end();
    }
}
