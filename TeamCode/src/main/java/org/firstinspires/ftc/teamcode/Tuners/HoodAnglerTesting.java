package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Systems.Hood.HoodAngler;

@Config
@TeleOp(group = "testing")
public class HoodAnglerTesting extends LinearOpMode {

    private HoodAngler hoodAngler;

    public static double POSITION;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        hoodAngler = new HoodAngler(hardwareMap, DeviceConstants.hoodServoDeviceName);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            hoodAngler.setPosition(POSITION);

            telemetry.addData("position", hoodAngler.getPosition());
            telemetry.update();
        }

    }
}
