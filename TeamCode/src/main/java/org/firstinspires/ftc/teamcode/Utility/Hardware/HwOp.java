package org.firstinspires.ftc.teamcode.Utility.Hardware;

/**
 * Every hardware operation the {@code Monitored*} wrappers track.
 *
 * <p>One enum covers all device types so per-operation statistics can live in a flat array
 * indexed by {@link #ordinal()} (no maps, no boxing on the hot path). Operations that exist on
 * several device types (e.g. {@code setPower} on motors and CR servos) share one constant.
 */
public enum HwOp {
    // ---- power / direction (DcMotorSimple: motors + CR servos) ----
    SET_POWER(Kind.WRITE, "setPower"),
    GET_POWER(Kind.READ, "getPower"),
    SET_DIRECTION(Kind.WRITE, "setDirection"),
    GET_DIRECTION(Kind.READ, "getDirection"),

    // ---- DcMotor ----
    SET_MODE(Kind.WRITE, "setMode"),
    GET_MODE(Kind.READ, "getMode"),
    SET_ZERO_POWER_BEHAVIOR(Kind.WRITE, "setZeroPowerBehavior"),
    GET_ZERO_POWER_BEHAVIOR(Kind.READ, "getZeroPowerBehavior"),
    SET_POWER_FLOAT(Kind.WRITE, "setPowerFloat"),
    GET_POWER_FLOAT(Kind.READ, "getPowerFloat"),
    SET_TARGET_POSITION(Kind.WRITE, "setTargetPosition"),
    GET_TARGET_POSITION(Kind.READ, "getTargetPosition"),
    IS_BUSY(Kind.READ, "isBusy"),
    GET_CURRENT_POSITION(Kind.READ, "getCurrentPosition"),
    SET_MOTOR_TYPE(Kind.WRITE, "setMotorType"),
    GET_MOTOR_TYPE(Kind.READ, "getMotorType"),

    // ---- DcMotorEx ----
    SET_MOTOR_ENABLE(Kind.WRITE, "setMotorEnable"),
    SET_MOTOR_DISABLE(Kind.WRITE, "setMotorDisable"),
    IS_MOTOR_ENABLED(Kind.READ, "isMotorEnabled"),
    SET_VELOCITY(Kind.WRITE, "setVelocity"),
    GET_VELOCITY(Kind.READ, "getVelocity"),
    SET_PIDF(Kind.WRITE, "setPIDFCoefficients"),
    GET_PIDF(Kind.READ, "getPIDFCoefficients"),
    SET_TARGET_POSITION_TOLERANCE(Kind.WRITE, "setTargetPositionTolerance"),
    GET_TARGET_POSITION_TOLERANCE(Kind.READ, "getTargetPositionTolerance"),
    GET_CURRENT(Kind.READ, "getCurrent"),
    SET_CURRENT_ALERT(Kind.WRITE, "setCurrentAlert"),
    GET_CURRENT_ALERT(Kind.READ, "getCurrentAlert"),
    IS_OVER_CURRENT(Kind.READ, "isOverCurrent"),

    // ---- Servo ----
    SET_POSITION(Kind.WRITE, "setPosition"),
    GET_POSITION(Kind.READ, "getPosition"),
    SCALE_RANGE(Kind.WRITE, "scaleRange"),

    // ---- PwmControl (ServoImplEx / CRServoImplEx) ----
    SET_PWM_RANGE(Kind.WRITE, "setPwmRange"),
    GET_PWM_RANGE(Kind.READ, "getPwmRange"),
    SET_PWM_ENABLE(Kind.WRITE, "setPwmEnable"),
    SET_PWM_DISABLE(Kind.WRITE, "setPwmDisable"),
    IS_PWM_ENABLED(Kind.READ, "isPwmEnabled"),
    SET_PULSE_WIDTH(Kind.WRITE, "setPulseWidth"),
    GET_PULSE_WIDTH(Kind.READ, "getPulseWidth"),

    // ---- IMU ----
    IMU_INITIALIZE(Kind.WRITE, "initialize"),
    IMU_RESET_YAW(Kind.WRITE, "resetYaw"),
    GET_YAW_PITCH_ROLL(Kind.READ, "getRobotYawPitchRollAngles"),
    GET_ORIENTATION(Kind.READ, "getRobotOrientation"),
    GET_QUATERNION(Kind.READ, "getRobotOrientationAsQuaternion"),
    GET_ANGULAR_VELOCITY(Kind.READ, "getRobotAngularVelocity"),

    // ---- DistanceSensor ----
    GET_DISTANCE(Kind.READ, "getDistance"),

    // ---- ColorSensor / NormalizedColorSensor ----
    GET_RED(Kind.READ, "red"),
    GET_GREEN(Kind.READ, "green"),
    GET_BLUE(Kind.READ, "blue"),
    GET_ALPHA(Kind.READ, "alpha"),
    GET_ARGB(Kind.READ, "argb"),
    GET_NORMALIZED_COLORS(Kind.READ, "getNormalizedColors"),
    ENABLE_LED(Kind.WRITE, "enableLed"),
    SET_GAIN(Kind.WRITE, "setGain"),
    GET_GAIN(Kind.READ, "getGain"),
    SET_I2C_ADDRESS(Kind.WRITE, "setI2cAddress"),
    GET_I2C_ADDRESS(Kind.READ, "getI2cAddress"),

    // ---- TouchSensor ----
    IS_PRESSED(Kind.READ, "isPressed"),
    GET_TOUCH_VALUE(Kind.READ, "getValue"),

    // ---- DigitalChannel ----
    GET_STATE(Kind.READ, "getState"),
    SET_STATE(Kind.WRITE, "setState"),

    // ---- AnalogInput / VoltageSensor ----
    GET_VOLTAGE(Kind.READ, "getVoltage"),
    GET_MAX_VOLTAGE(Kind.READ, "getMaxVoltage");

    /** Whether an operation pulls data from the device or pushes data to it. */
    public enum Kind { READ, WRITE }

    public final Kind kind;
    /** The SDK method name, used in reports and the event log. */
    public final String label;

    HwOp(Kind kind, String label) {
        this.kind = kind;
        this.label = label;
    }

    public boolean isRead() {
        return kind == Kind.READ;
    }

    private static final HwOp[] VALUES = values();

    /** Allocation-free {@code values()[i]}. */
    public static HwOp fromOrdinal(int ordinal) {
        return VALUES[ordinal];
    }

    public static int count() {
        return VALUES.length;
    }
}
