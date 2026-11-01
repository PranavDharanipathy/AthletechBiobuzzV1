package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

@Config
@TeleOp (group = "tuning")
public class FlywheelTuner extends LinearOpMode {

    public static int LOOP_TIME = 60;

    public enum TUNING_STAGES {

        PIDVS /*1st*/, STABILITY
    }

    public static TUNING_STAGES TUNING_STAGE = TUNING_STAGES.PIDVS;

    public static double KP_FAR = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpFar;
    public static double KP_CLOSE = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kpClose;
    public static double KI_FAR = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiFar;
    public static double KI_CLOSE = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kiClose;
    public static double KD = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kd;
    public static double KV = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kv;
    public static double KS = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.ks;
    public static double KS_SIGN_WIDTH = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.ksSignWidth;
    public static double KI_SMASH = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.kISmash;
    public static double I_SWITCH = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.iSwitch;
    public static double P_SWITCH = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.pSwitch;

    public static double D_MIN = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minD, D_MAX = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxD;
    public static double I_MIN = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minI, I_MAX = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxI;
    public static double P_MIN = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.minP, P_MAX = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS.maxP;

    public static double VELOCITY;

    private Flywheel flywheel;
    private FlywheelPIDVSCoefficients coefficients;

    private VoltageSensor batteryVoltageSensor;


    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(12);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx flywheelMotor = new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false);

        flywheel = new Flywheel(flywheelMotor);
        flywheel.initVoltageSensor(hardwareMap);

        coefficients = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        flywheel.setVelocityPIDVSCoefficients(coefficients);

        if (isStopRequested()) return;
        waitForStart();

        flywheel.reset();

        while (opModeIsActive()) {

            HardwareMonitor.get().update();

            coefficients.updateCoefficients(
                    KP_FAR, KP_CLOSE,
                    KI_FAR, KI_CLOSE,
                    KD,
                    KV,
                    KS,
                    KS_SIGN_WIDTH,
                    I_SWITCH,
                    P_SWITCH,
                    KI_SMASH,
                    P_MIN, P_MAX,
                    I_MIN, I_MAX,
                    D_MIN, D_MAX
            );

            telemetry.addData("voltage", batteryVoltageSensor.getVoltage());

            flywheel.setVelocityPIDVSCoefficients(coefficients);

            flywheel.setVelocity(VELOCITY, true);

            flywheel.update();

            telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
            telemetry.addData("Current Velocity", flywheel.getCurrentVelocity());
            telemetry.addData("Velocity Estimate", flywheel.getVelocityEstimate());
            telemetry.addData("Window Velocity (100 ms)", flywheel.getWindowVelocity());
            telemetry.addData("Ready (within 1 %)", flywheel.isReady());
            telemetry.addData("Error", flywheel.getError());
            telemetry.addData("ki", flywheel.ki);
            telemetry.addData("p", flywheel.getPIDVS()[0]);
            telemetry.addData("i", flywheel.getPIDVS()[1]);
            telemetry.addData("d", flywheel.getPIDVS()[2]);
            telemetry.addData("v", flywheel.getPIDVS()[3]);
            telemetry.addData("s", flywheel.getPIDVS()[4]);

            if (TUNING_STAGE == TUNING_STAGES.STABILITY) {
                telemetry.addData("Is At Velocity And Stable", flywheel.isAtVelocityAndStable(FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR, FlywheelConstants.FLYWHEEL_STABILITY_MARGIN_OF_ERROR));
            }

            telemetry.addData("Flywheel motor power", flywheel.getMotorPower());
            telemetry.addData("Is motor enabled", flywheel.getPIDEnabled());
            telemetry.update();

            sleep(LOOP_TIME);
        }
    }
}
