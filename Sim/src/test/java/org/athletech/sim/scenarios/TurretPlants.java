package org.athletech.sim.scenarios;

import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

//the robot's turret: lag, dead time, start delay, torque limits from the STATIONARY run 2028-05-07; speed per volt and friction from the
//Turret Autotuner 2028-05-12 (150 deg/s per V, 1.18 V, breakaway ramps 0.95-1.24 V). Not TurretConstants: the controller's belief can be wrong
final class TurretPlants {

    private TurretPlants() {}

    static final double TIME_CONSTANT = 0.046, DEAD_TIME = 0.0361, START_DELAY = 0.0828;
    //the servos ignore commands this long after an OpMode starts
    static final double SERVO_WAKE = TurretScenario.SERVO_WAKE;
    static final double MAX_SPEED = 26233, MAX_ACCELERATION = 118456, COAST_DECELERATION = 71702;
    static final double GAIN = 10220.8, KINETIC = 1.18, BREAKAWAY = 1.24;

    static CrServoTurret.Params params(double gainScale, double frictionScale) {
        return params(gainScale, frictionScale, frictionScale);
    }

    static CrServoTurret.Params params(double gainScale, double kineticScale, double breakawayScale) {

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        CrServoTurret.Params p = new CrServoTurret.Params(GAIN * gainScale, TIME_CONSTANT, DEAD_TIME, KINETIC * kineticScale, BREAKAWAY * breakawayScale, 0.44 * tpd, MAX_SPEED, MAX_ACCELERATION);
        p.startDelay = START_DELAY;
        p.coastDecel = COAST_DECELERATION;
        p.neutralDecel = COAST_DECELERATION;
        return p;
    }

    static CrServoTurret measured(long seed) {
        return new CrServoTurret(params(1, 1), seed);
    }
}
