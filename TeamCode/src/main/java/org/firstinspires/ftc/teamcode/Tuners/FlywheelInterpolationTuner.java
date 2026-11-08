package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Field.Garden;
import org.firstinspires.ftc.teamcode.Field.Goal;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelVelocityModel;
import org.firstinspires.ftc.teamcode.Systems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpBaseOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/// collects the flywheel speed table (ShooterConstants.FLYWHEEL_DISTANCES / FLYWHEEL_VELOCITIES): drive somewhere, find the
/// speed that scores, record (distance, speed), repeat. The turret aims at the goal exactly like a match (the Shooter, its
/// speed held at yours), and the distance is the match's: turret to the goal's CELL point. One goal is enough: the table is
/// distances, so blue (the same field flipped) uses it too. RED by default
@Config
@TeleOp(name = "Flywheel Interpolation Tuner", group = "tuning")
public class FlywheelInterpolationTuner extends TeleOpBaseOpMode {

    public static Alliance ALLIANCE = Alliance.RED;
    public static double FLYWHEEL_VELOCITY = 1450;
    public static double VELOCITY_STEP = 10;
    //a point is only recorded with the robot this still (in/s, deg/s): its distance is where the shots came from
    public static double MAX_RECORD_SPEED = 2, MAX_RECORD_YAW = 5;

    private final List<Double> distances = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final FlywheelVelocityModel collected = new FlywheelVelocityModel();
    private final Shooter shooter = new Shooter();

    private Drivetrain drivetrain;
    private Intake intakeComponent;
    private Telemetry telemetry;
    private boolean localized, spinning = true, showDataset;
    private double distance = Double.NaN;
    private String note = "";

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeComponents(ALLIANCE);
        applyComponentTraits();
        drivetrain = createDrivetrain();
        intakeComponent = new Intake(intake);
        shooter.provideComponents(flywheel, turret, follower);
        turret.setVolts(0, 0);
        recorder.channel("interp", "velocity", "ticks/s", "speed being tried", () -> FLYWHEEL_VELOCITY);
        recorder.channel("interp", "points", "", "recorded so far", () -> distances.size());
        record(shooter, null, intakeComponent, drivetrain);
        setUpLynxModule();
    }

    @Override
    public void init_loop() {

        telemetry.addLine("FLYWHEEL INTERPOLATION TUNER (" + ALLIANCE + " goal)");
        telemetry.addLine();
        telemetry.addLine("Before START:");
        telemetry.addLine(" - Turret pointing straight forward (that's its home).");
        telemetry.addLine(" - Balls out of the way of the flywheel.");
        telemetry.addLine();
        telemetry.addLine("After START:");
        telemetry.addLine(" 1. Drive into the " + ALLIANCE + " GARDEN corner: front against the GARDEN's wall, side against the other wall.");
        telemetry.addLine(" 2. Press the MAIN button (PS / guide) to localize. Only then does it aim and spin up.");
        telemetry.addLine(" 3. Drive to a spot and stop. LEFT TRIGGER (hold) shoots.");
        telemetry.addLine(" 4. DPAD LEFT/RIGHT: speed -/+ " + VELOCITY_STEP + " (or FLYWHEEL_VELOCITY on the Dashboard) until shots land in the middle of the CELL.");
        telemetry.addLine(" 5. DPAD UP records (distance, speed), robot still. DPAD DOWN removes the last one.");
        telemetry.addLine(" 6. Repeat from close to far (more points = better fit). Y shows the table to copy into ShooterConstants.");
        telemetry.addLine();
        telemetry.addLine("B: flywheel on/off. MAIN again: re-localize. Distances are turret to the CELL point the match aims at (CLOSE if y > 72).");
        telemetry.update();
    }

    @Override
    public void start() {
        flywheel.reset();
    }

    @Override
    public void loop() {

        clearCacheOfLynxModule();
        controller1.getInformation();
        follower.update();
        drivetrain.update();

        if (controller1.main_buttonHasJustBeenPressed) localize();
        if (!localized) {
            telemetry.addLine("Drive into the " + ALLIANCE + " GARDEN corner (front on the GARDEN's wall, side on the other wall), then press MAIN.");
            telemetry.addData("Pose now", poseText(follower.pose()));
            telemetry.update();
            return;
        }

        if (controller1.bHasJustBeenPressed) spinning = !spinning;
        if (controller1.dpad_rightHasJustBeenPressed) FLYWHEEL_VELOCITY += VELOCITY_STEP;
        if (controller1.dpad_leftHasJustBeenPressed) FLYWHEEL_VELOCITY = Math.max(0, FLYWHEEL_VELOCITY - VELOCITY_STEP);
        if (controller1.yHasJustBeenPressed) showDataset = !showDataset;

        //the match aim, at your speed
        shooter.manualFlywheelVelocity = FLYWHEEL_VELOCITY;
        shooter.updateAutonomous(spinning);
        distance = Goal.isSet(shooter.goalCoordinate) ? shooter.turretPose.distance(shooter.goalCoordinate) : Double.NaN;

        boolean shoot = controller1.left_trigger(GeneralConstants.TRIGGER_THRESHOLD) && spinning;
        blocker.setState(shoot ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);
        intakeComponent.setState(shoot ? Intake.IntakeState.INTAKE : Intake.IntakeState.IDLE);
        //a component Intake (no gamepad): its update() is the TeleOp one and reads the triggers, so run the motor's speed loop
        intake.update();

        if (controller1.dpad_upHasJustBeenPressed) record();
        else if (controller1.dpad_downHasJustBeenPressed && !distances.isEmpty()) {
            distances.remove(distances.size() - 1);
            velocities.remove(velocities.size() - 1);
            note = "removed the last point";
        }

        if (showDataset) showDataset();
        else showLive();
        telemetry.update();
    }

    private void localize() {

        follower.setPose(Garden.forAlliance(alliance.getAlliance()).relocalizationPose());
        if (!localized) shooter.start(alliance);
        localized = true;
        note = "localized at " + poseText(follower.pose());
    }

    private void record() {

        Velocity v = follower.velocity();
        if (Math.hypot(v.vx, v.vy) > MAX_RECORD_SPEED || Math.abs(Math.toDegrees(v.omega)) > MAX_RECORD_YAW) {
            note = "NOT recorded: stop the robot first";
            return;
        }
        if (!spinning || distance != distance) {
            note = "NOT recorded: flywheel off or no goal";
            return;
        }
        distances.add(distance);
        velocities.add(FLYWHEEL_VELOCITY);
        note = String.format(Locale.US, "recorded %.1f in -> %.0f ticks/s (%d points)", distance, FLYWHEEL_VELOCITY, distances.size());
    }

    private void showLive() {

        telemetry.addData("Note", note);
        telemetry.addData("Distance to goal (in)", "%.1f", distance);
        telemetry.addData("Aiming at", Goal.isClose(follower.pose()) ? "CLOSE cell" : "FAR cell");
        telemetry.addData("Flywheel target / actual", "%.0f / %.0f%s", FLYWHEEL_VELOCITY, flywheel.getCurrentVelocity(), spinning ? "" : " (OFF, B)");
        telemetry.addData("Turret on target", turret.isOnTarget());
        telemetry.addData("Pose", poseText(follower.pose()));
        telemetry.addData("Points", distances.size());
        telemetry.addLine("LT shoot | DPAD L/R speed | DPAD UP record | DPAD DOWN undo | Y table | B flywheel | MAIN re-localize");
    }

    private void showDataset() {

        telemetry.addData("Note", note);
        telemetry.addData("FLYWHEEL_DISTANCES", list(distances, "%.1f"));
        telemetry.addData("FLYWHEEL_VELOCITIES", list(velocities, "%.0f"));
        collected.velocity(distances, velocities, ShooterConstants.FLYWHEEL_FIT_DEGREE, 0, 0);
        telemetry.addData("Fit through these", collected.describe());
        Shooter.getFlywheelTargetVelocityFromInterpolation(distance);
        telemetry.addData("Fit in ShooterConstants now", Shooter.flywheelModel().describe());
        telemetry.addLine("Copy both lists into ShooterConstants (same order). Y: back.");
    }

    private static String list(List<Double> values, String format) {

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < values.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(Locale.US, format, values.get(i)));
        }
        return sb.toString();
    }

    private static String poseText(Pose p) {
        return String.format(Locale.US, "x %.1f  y %.1f  heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    @Override
    public void stop() {

        flywheel.setVelocity(0, true);
        flywheel.update();
        super.stop();
    }
}
