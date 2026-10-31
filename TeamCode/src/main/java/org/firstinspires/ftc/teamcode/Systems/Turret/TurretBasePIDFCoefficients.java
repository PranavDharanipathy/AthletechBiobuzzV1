package org.firstinspires.ftc.teamcode.Systems.Turret;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KPS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KDS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KFS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_PD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_FEEDFORWARD_POSITIONS;

import org.firstinspires.ftc.teamcode.Utility.InterpolationData;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

/// Easier usage of the coefficients for the left and right sides of the robot.
public class TurretBasePIDFCoefficients {

    public double kpFar, kpClose;
    public double lkiFar, rkiFar;
    public double lkiClose, rkiClose;
    public double kdFar, kdClose;
    public double kHold;

    public double pSwitch;
    public double[] iSwitch;
    public double dSwitch;

    public double lkISmash, rkISmash;

    public double[] dActivation;

    public double holdDecay;

    public double minI, maxI;

    private boolean tuning = false;

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    public TurretBasePIDFCoefficients withTuning(boolean tuning) {
        this.tuning = tuning;
        return this;
    }

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    public void setTuning(boolean tuning) {
        this.tuning = tuning;
    }

    /// Index 0: left
    /// <p>
    /// Index 1: right
    public TurretBasePIDFCoefficients(
            double kpFar,
            double kpClose,
            double[] kiFar,
            double[] kiClose,
            double kdFar,
            double kdClose,
            double kHold,
            double pSwitch,
            double[] iSwitch,
            double dSwitch,
            double[] kISmash,
            double[] dActivation,
            double holdDecay,
            double minI,
            double maxI
    ) {

        this.kpFar = kpFar;
        this.kpClose = kpClose;

        lkiFar = kiFar[0];
        rkiFar = kiFar[1];

        lkiClose = kiClose[0];
        rkiClose = kiClose[1];

        this.kdFar = kdFar;
        this.kdClose = kdClose;

        this.kHold = kHold;

        this.pSwitch = pSwitch;

        this.iSwitch = iSwitch;

        this.dSwitch = dSwitch;

        lkISmash = kISmash[0];
        rkISmash = kISmash[1];

        this.dActivation = dActivation;

        this.holdDecay = holdDecay;

        this.minI = minI;
        this.maxI = maxI;
    }

    public enum TurretSide {

        LEFT, RIGHT;

        public static TurretSide getSide(double targetPosition, double startPosition, boolean reversed) {

            boolean sideCondition = reversed ? targetPosition > startPosition : targetPosition < startPosition;

            return sideCondition ? TurretSide.RIGHT : TurretSide.LEFT;
        }
    }

    public double kp(double targetPosition, double currentPosition, double startPosition) {

        double error = targetPosition - currentPosition;
        if (tuning) return Math.abs(error) < pSwitch ? kpClose : kpFar;

        double reZeroedTargetPosition = targetPosition - startPosition;

        if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_PD_POSITIONS.get(0), TURRET_PD_POSITIONS.get(TURRET_PD_POSITIONS.size() - 1))) {
            return getKpFromInterpolation(reZeroedTargetPosition);
        }

        if (reZeroedTargetPosition < TURRET_PD_POSITIONS.get(0)) {
            return TURRET_KPS.get(0);
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return TURRET_KPS.get(TURRET_KPS.size() - 1);
    }

    public double kiFar(TurretSide side) {
        return side == TurretSide.LEFT ? lkiFar : rkiFar;
    }

    public double kiClose(TurretSide side) {
        return side == TurretSide.LEFT ? lkiClose : rkiClose;
    }

    public double kd(double targetPosition, double currentPosition, double startPosition) {

        double error = targetPosition - currentPosition;
        if (tuning) return (Math.abs(error) < dSwitch ? kdClose : kdFar);

        double reZeroedTargetPosition = targetPosition - startPosition;

        if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_PD_POSITIONS.get(0), TURRET_PD_POSITIONS.get(TURRET_PD_POSITIONS.size() - 1))) {
            return getKdFromInterpolation(reZeroedTargetPosition);
        }

        if (reZeroedTargetPosition < TURRET_PD_POSITIONS.get(0)) {
            return TURRET_KDS.get(0);
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return TURRET_KDS.get(TURRET_KDS.size() - 1);
    }

    public double kHold(double targetPosition, double startPosition) {

        if (tuning) return kHold;

        double reZeroedTargetPosition = targetPosition - startPosition;

        if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_FEEDFORWARD_POSITIONS.get(0), TURRET_FEEDFORWARD_POSITIONS.get(TURRET_FEEDFORWARD_POSITIONS.size() - 1))) {
            return getKfFromInterpolation(reZeroedTargetPosition);
        }

        if (reZeroedTargetPosition < TURRET_FEEDFORWARD_POSITIONS.get(0)) {
            return TURRET_KFS.get(0);
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return TURRET_KFS.get(TURRET_KFS.size() - 1);
    }

    public double kISmash(TurretSide side) {
        return side == TurretSide.LEFT ? lkISmash : rkISmash;
    }

    private double getKfFromInterpolation(double reZeroedTargetPosition) {

        //converting list to array
        double[] turretFeedforwardTargetPositions = TURRET_FEEDFORWARD_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretFeedforwardTargetPositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        double kf0 = TURRET_KFS.get(TURRET_FEEDFORWARD_POSITIONS.indexOf(targetPosition0));
        double kf1 = TURRET_KFS.get(TURRET_FEEDFORWARD_POSITIONS.indexOf(targetPosition1));

        //returning kf
        return MathUtil.interpolateLinear(

                reZeroedTargetPosition,

                new InterpolationData(
                        new double[] {targetPosition0, kf0},
                        new double[] {targetPosition1, kf1}
                )
        );

    }

    private double getKpFromInterpolation(double reZeroedTargetPosition) {

        //converting list to array - same positions are used for p and d interpolations
        double[] turretProportionalPositions = TURRET_PD_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretProportionalPositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        double kp0 = TURRET_KPS.get(TURRET_PD_POSITIONS.indexOf(targetPosition0));
        double kp1 = TURRET_KPS.get(TURRET_PD_POSITIONS.indexOf(targetPosition1));

        //returning kp
        return MathUtil.interpolateLinear(

                reZeroedTargetPosition,

                new InterpolationData(
                        new double[] {targetPosition0, kp0},
                        new double[] {targetPosition1, kp1}
                )
        );

    }

    private double getKdFromInterpolation(double reZeroedTargetPosition) {

        //converting list to array - same positions are used for p and d interpolations
        double[] turretDerivativePositions = TURRET_PD_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretDerivativePositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        double kd0 = TURRET_KDS.get(TURRET_PD_POSITIONS.indexOf(targetPosition0));
        double kd1 = TURRET_KDS.get(TURRET_PD_POSITIONS.indexOf(targetPosition1));

        //returning kd
        return MathUtil.interpolateLinear(

                reZeroedTargetPosition,

                new InterpolationData(
                        new double[] {targetPosition0, kd0},
                        new double[] {targetPosition1, kd1}
                )
        );

    }

    private double[] getKpAndKdFromInterpolation(double reZeroedTargetPosition) {

        //converting list to array - same positions are used for p and d interpolations
        double[] turretPDPositions = TURRET_PD_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretPDPositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        final InterpolationData pData = new InterpolationData(
                new double[] {targetPosition0, TURRET_KPS.get(TURRET_PD_POSITIONS.indexOf(targetPosition0))},
                new double[] {targetPosition1, TURRET_KPS.get(TURRET_PD_POSITIONS.indexOf(targetPosition1))}
        );

        final InterpolationData dData = new InterpolationData(
                new double[] {targetPosition0, TURRET_KDS.get(TURRET_PD_POSITIONS.indexOf(targetPosition0))},
                new double[] {targetPosition1, TURRET_KDS.get(TURRET_PD_POSITIONS.indexOf(targetPosition1))}
        );

        //returning kp and kd
        return new double[] {

                //p
                MathUtil.interpolateLinear(reZeroedTargetPosition, pData),

                //d
                MathUtil.interpolateLinear(reZeroedTargetPosition, dData),
        };
    }

    /// @return A double array with index 0 being kp and index 1 being kd
    public double[] kpAndKd(double targetPosition, double currentPosition, double startPosition) {

        if (tuning) {
            double errorMag = Math.abs(targetPosition - currentPosition);
            return new double[] {
                    (errorMag < pSwitch ? kpClose : kpFar),
                    (errorMag < dSwitch ? kdClose : kdFar)
            };
        }

        double reZeroedTargetPosition = targetPosition - startPosition;

        if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_PD_POSITIONS.get(0), TURRET_PD_POSITIONS.get(TURRET_PD_POSITIONS.size() - 1))) {
            return getKpAndKdFromInterpolation(reZeroedTargetPosition);
        }

        if (reZeroedTargetPosition < TURRET_PD_POSITIONS.get(0)) {
            return new double[] {
                    TURRET_KPS.get(0),
                    TURRET_KDS.get(0)
            };
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return new double[] {
                TURRET_KPS.get(TURRET_KPS.size() - 1),
                TURRET_KDS.get(TURRET_KDS.size() - 1)
        };
    }
}
