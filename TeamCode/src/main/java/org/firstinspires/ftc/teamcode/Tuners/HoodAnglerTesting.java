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

        hoodAngler = new HoodAngler(hardwareMap, DeviceConstants.leftHoodAnglerServoDeviceName, DeviceConstants.rightHoodAnglerServoDeviceName);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            hoodAngler.setPosition(POSITION);

            telemetry.addData("position", hoodAngler.getPosition());
            telemetry.addData("left / right servo (with trims)", "%.4f / %.4f",
                    hoodAngler.getServoPositions()[0], hoodAngler.getServoPositions()[1]);
            telemetry.addLine("Trims: HoodConstants.LEFT_HOOD_TRIM / RIGHT_HOOD_TRIM (dashboard)");
            telemetry.update();
        }

    }
}