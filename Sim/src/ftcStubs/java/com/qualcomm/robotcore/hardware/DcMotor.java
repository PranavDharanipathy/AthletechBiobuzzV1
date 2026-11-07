package com.qualcomm.robotcore.hardware;

public interface DcMotor extends DcMotorSimple {

    enum RunMode { RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER }

    enum ZeroPowerBehavior { UNKNOWN, BRAKE, FLOAT }

    void setMode(RunMode mode);

    RunMode getMode();

    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);

    int getCurrentPosition();
}
