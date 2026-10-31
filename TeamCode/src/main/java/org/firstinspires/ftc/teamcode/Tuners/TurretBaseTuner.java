package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_POSITIONAL_COEFFICIENTS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBasePIDFCoefficients;

@Config
@TeleOp(group = "tuning")
public class TurretBaseTuner extends OpMode {

    private TurretBase turret;

    public static long LOOP_TIME = 60;

    public static double KP_FAR = TURRET_POSITIONAL_COEFFICIENTS.kpFar;
    public static double KP_CLOSE = TURRET_POSITIONAL_COEFFICIENTS.kpClose;
    public static double[] KI_FAR = {TURRET_POSITIONAL_COEFFICIENTS.lkiFar, TURRET_POSITIONAL_COEFFICIENTS.rkiFar};
    public static double[] KI_CLOSE = {TURRET_POSITIONAL_COEFFICIENTS.lkiClose, TURRET_POSITIONAL_COEFFICIENTS.rkiClose};
    public static double KD_FAR = TURRET_POSITIONAL_COEFFICIENTS.kdFar;
    public static double KD_CLOSE = TURRET_POSITIONAL_COEFFICIENTS.kdClose;
    public static double KHOLD = TURRET_POSITIONAL_COEFFICIENTS.kHold;

    public static double P_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.pSwitch;

    public static double[] I_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.iSwitch;

    public static double D_SWITCH = TURRET_POSITIONAL_COEFFICIENTS.dSwitch;

    public static double[] KI_SMASH = {TURRET_POSITIONAL_COEFFICIENTS.lkISmash, TURRET_POSITIONAL_COEFFICIENTS.rkISmash};

    public static double[] D_ACTIVATION = TURRET_POSITIONAL_COEFFICIENTS.dActivation;

    public static double HOLD_DECAY = TURRET_POSITIONAL_COEFFICIENTS.holdDecay;

    public static double MIN_I = TURRET_POSITIONAL_COEFFICIENTS.minI, MAX_I = TURRET_POSITIONAL_COEFFICIENTS.maxI;
    public static double TARGET_POSITION;

    public static String PD_MODE = "00";

    private Telemetry telemetry;

    private TurretBasePIDFCoefficients coefficientsFromDashboard() {
        return new TurretBasePIDFCoefficients(
                KP_FAR,
                KP_CLOSE,
                KI_FAR,
                KI_CLOSE,
                KD_FAR,
                KD_CLOSE,
                KHOLD,
                P_SWITCH,
                I_SWITCH,
                D_SWITCH,
                KI_SMASH,
                D_ACTIVATION,
                HOLD_DECAY,
                MIN_I, MAX_I
        );
    }

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretBase(hardwareMap);
        turret.setPositionalCoefficients(coefficientsFromDashboard());

        TARGET_POSITION = turret.startPosition;
    }

    @Override
    public void loop() {

        turret.setPdInterpolationMode(TurretBase.PD_INTERPOLATION_MODE.fromString(PD_MODE));

        turret.setPosition(TARGET_POSITION);
        turret.setPositionalCoefficients(coefficientsFromDashboard());
        turret.setTuning(true);
        turret.update();

        telemetry.addData("kp", "%.8f", turret.kp);
        telemetry.addData("ki", "%.8f", turret.ki);
        telemetry.addData("kd", "%.8f", turret.kd);
        telemetry.addData("kHold", "%.8f", turret.kHold);
        telemetry.addData("p", "%.5f", turret.p);
        telemetry.addData("i", "%.5f", turret.i);
        telemetry.addData("d", "%.5f", turret.d);
        telemetry.addData("f", "%.5f", turret.f);

        telemetry.addData("position error", turret.getError());
        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("target position", turret.getTargetPosition());

        telemetry.addData("current velocity", turret.getVelocity());

        telemetry.addData("power", turret.getPower());
        telemetry.addData("left servo power", turret.getServoPowers()[0]);
        telemetry.addData("right servo power", turret.getServoPowers()[1]);
        telemetry.addData("start position", turret.startPosition);
        telemetry.update();

        sleep(LOOP_TIME);

    }
}
