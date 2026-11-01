package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;

    private Direction direction;
    public Encoder(DcMotorEx motor) {

        this.motor = motor;

        this.direction = Direction.FORWARD;
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        return motor.getCurrentPosition() * multiplier;
    }

    /**
     * Hub-measured velocity in ticks/s. The REV hub reports it as a 16-bit signed value
     * ({@code short[] velocities} in LynxGetBulkInputDataResponse), so it wraps at ±32768 ticks/s.
     * Use {@link #getInternalVelocity(double)} for fast, high-resolution encoders.
     */
    public double getInternalVelocity() {
        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /** One full wrap of the hub's 16-bit velocity register, ticks/s. */
    public static final double HUB_VELOCITY_WRAP = 65536;

    /**
     * Undoes the 16-bit wrap of a hub velocity, using any independent estimate in the same sign
     * convention (for example Δposition/Δt) that is within ±32768 ticks/s of the truth.
     * Returns the input unchanged if the estimate is not finite.
     */
    public static double correctVelocityOverflow(double hubVelocity, double estimate) {
        if (!Double.isFinite(hubVelocity) || !Double.isFinite(estimate)) return hubVelocity;
        return hubVelocity + Math.round((estimate - hubVelocity) / HUB_VELOCITY_WRAP) * HUB_VELOCITY_WRAP;
    }

    /** Hub velocity with the 16-bit wrap removed, using {@code estimate} (ticks/s) to pick the wrap. */
    public double getInternalVelocity(double estimate) {
        return correctVelocityOverflow(getInternalVelocity(), estimate);
    }
    private double filteredVelocity = 0;

    private double kalmanGain = 0;

    private double p = 1; //prediction step
    private double q = 0; //uncertainty step
    private double r = 0; //encoder uncertainty

    private double outlierSigma = 1;
    private double kRInflation = 1; //when outlier is caught

    private boolean velocityKFInitialized = false;

    /// @param parameters should contain 4 values in order of q, r, outlierSigma, kRInflation
    public void setupVelocityKalmanFilter(double[] parameters) {

        velocityKFInitialized = true;

        if (q != parameters[0] || r != parameters[1]) p = 1;

        q = parameters[0];
        r = parameters[1];

        outlierSigma = parameters[2];
        kRInflation = parameters[3];
    }

    /// Uses Kalman Filter.
    /// @param velocityEstimate (curr_ticks - prev_ticks) / dt
    public void runVelocityCalculation(double velocityEstimate) {

        if (!velocityKFInitialized) throw new RuntimeException("Velocity Kalman Filter is not being used!");

        double innovation = velocityEstimate - filteredVelocity;

        p += q;

        double jump = p + r;

        double correctedR = r; //deals with outliers
        if (Math.abs(innovation) > outlierSigma * Math.sqrt(jump)) {
            correctedR = kRInflation * r;
        }

        double kalmanGainDenominator = p + correctedR;
        kalmanGain = kalmanGainDenominator != 0 ? p / kalmanGainDenominator : 0;

        filteredVelocity += kalmanGain * innovation;

        p *= (1 - kalmanGain);
    }

    /// In ticks per second.
    /// Using Kalman Filter.
    /// @return The filtered velocity -- will return 0 if Kalman Filter wasn't initialized.
    public double getFilteredVelocity() {
        return filteredVelocity;
    }
}
