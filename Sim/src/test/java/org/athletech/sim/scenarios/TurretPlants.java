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
    //autotune 20280513-222052 (the one before it read 1.18 V friction, so it drifts)
    static final double GAIN = 11051.9, KINETIC = 0.98, BREAKAWAY = 1.04;

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
