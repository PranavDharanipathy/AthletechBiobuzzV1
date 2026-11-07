package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Encoder;

@Config
@TeleOp (group = "testing")
public class EncoderTesting extends OpMode {

    public static String DEVICE_NAME = "";

    public static Encoder.Direction DIRECTION = Encoder.Direction.FORWARD;

    private Telemetry telemetry;

    private Encoder encoder;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, DEVICE_NAME));
        encoder.setDirection(DIRECTION);

    }

    @Override
    public void loop() {

        telemetry.addData("position", encoder.getCurrentPosition());
        telemetry.update();
    }
}
