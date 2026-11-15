package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConstants {

    public static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double INTAKE_VELOCITY = 2300;
    public static double REVERSE_INTAKE_VELOCITY = -1000;

    public static double[] INTAKE_PDF_COEFFICIENTS = {0.00015, 0, 0.0005};

    public static double[] FIARM_RETRACT = {0, 0};
    public static double[] FIARM_DEPLOY = {0.41, 0.41};

}
