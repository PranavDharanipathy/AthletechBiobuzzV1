package org.firstinspires.ftc.teamcode.Testing.ShotExperiment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.List;

/** Controlled stationary comparison; deliberately separate from match TeleOp and Claude's turret work. */
@Config
@TeleOp(name="Pollen Shot Experiment", group="testing")
public final class PollenShotExperiment extends OpMode {
    // Dashboard edits apply on X with RB released. Hood and turret are mechanically fixed; no servo access.
    public static boolean HARDWARE_CONFIRMED=false;
    public static double MEASURED_LAUNCH_DEG=-1;
    public static double TARGET_RPM=0, RELEASE_DISTANCE_FT=6, FLYWHEEL_TICKS_PER_REV=28;
    public static double FEED_POWER=0;
    public static double RPM_TOLERANCE=75, MAX_RPM_PER_SECOND=400, STABLE_SECONDS=.35;
    public static double FEED_SECONDS=.12;
    public static double RECOVERY_SECONDS=.8, MAX_LOOP_SECONDS=.1, MIN_VOLTS=10.5;
    public static double MAX_DRIVE_TICKS_PER_SECOND=20, MAX_TURRET_TICKS_PER_SECOND=100;

    private final ExperimentFlywheel controller=new ExperimentFlywheel();
    private DcMotorEx motor, intake, turretEncoder;
    private DcMotorEx[] drive;
    private VoltageSensor battery;
    private List<LynxModule> hubs;
    private ShotGate gate;
    private double targetRpm, ticksPerRev, angle, distanceFt, feedPower;
    private double maxDrive, maxTurret, lastLoop=Double.NaN, lastLog=-1;
    private boolean lastX, lastUp, lastDown, lastLeft, lastRight, lastA, lastY;
    private int labeledAttempt, stays, exits, misses, rims, rowCount, batch;
    private String configCsv=",,,,,,,,,";
    private String status="Set verified Dashboard values; START; release RB and press X", outcome="NONE";
    private String lastState="", motorFault="";
    private double armStarted=Double.NaN;
    private File logFile;
    private final StringBuilder log=new StringBuilder(32768);

    @Override public void init() {
        // No servo lookup or command in INIT, loop, apply, or STOP.
        motor=new MonitoredDcMotorEx(hardwareMap,DeviceConstants.flywheelMotorDeviceName,false);
        motor.setPower(0);
        motor.setDirection(FlywheelConstants.FLYWHEEL_MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake=new MonitoredDcMotorEx(hardwareMap,DeviceConstants.intakeMotorDeviceName,false);
        intake.setPower(0); intake.setDirection(IntakeConstants.INTAKE_MOTOR_DIRECTION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretEncoder=new MonitoredDcMotorEx(hardwareMap,DeviceConstants.turretEncoderDeviceName,false);
        drive=new DcMotorEx[] {
            new MonitoredDcMotorEx(hardwareMap,DeviceConstants.leftFrontMotorDeviceName,false),
            new MonitoredDcMotorEx(hardwareMap,DeviceConstants.rightFrontMotorDeviceName,false),
            new MonitoredDcMotorEx(hardwareMap,DeviceConstants.leftBackMotorDeviceName,false),
            new MonitoredDcMotorEx(hardwareMap,DeviceConstants.rightBackMotorDeviceName,false)};
        battery=hardwareMap.voltageSensor.iterator().next();
        hubs=hardwareMap.getAll(LynxModule.class);
        stopActuators();
        log.append("time_s,dt_s,batch,attempt,state,target_rpm,measured_rpm,volts,flywheel_power,drive_tps,turret_tps,launch_deg,distance_ft,ticks_per_rev,feed_power,outcome,rpm_tolerance,max_rpm_per_s,stable_s,feed_s,recovery_s,maxloop_s,min_volts,max_drive_tps,max_turret_tps\n");
        logFile=new File(new File(AppUtil.FIRST_FOLDER,"data/ShotExperiments"),"pollen-"+System.currentTimeMillis()+".csv");
        telemetry.addLine(status); telemetry.addLine("Secure hood/turret manually. Intake alone must prevent early ball release.");
    }
    private static boolean between(double v,double low,double high) {
        return Double.isFinite(v) && v>=low && v<=high;
    }
    private void apply() {
        stopActuators(); gate=null; motorFault=""; armStarted=Double.NaN;
        // Confirm hood/turret are physically secured and intake alone prevents uncommanded release.
        if (!HARDWARE_CONFIRMED || !between(MEASURED_LAUNCH_DEG,31,54)
                || !between(TARGET_RPM,500,5000) || !between(FLYWHEEL_TICKS_PER_REV,1,1000)
                || !between(FEED_POWER,.01,1) || !between(RELEASE_DISTANCE_FT,4,12)
                || !between(MAX_DRIVE_TICKS_PER_SECOND,0,100)
                || !between(MAX_TURRET_TICKS_PER_SECOND,0,200)) {
            status="Invalid/unverified setup: see docs/POLLEN_SHOT_EXPERIMENT.md"; return;
        }
        try {
            ShotGate.Settings settings=new ShotGate.Settings(RPM_TOLERANCE,MAX_RPM_PER_SECOND,
                    STABLE_SECONDS,MAX_LOOP_SECONDS,MIN_VOLTS,FEED_SECONDS,RECOVERY_SECONDS);
            targetRpm=TARGET_RPM; ticksPerRev=FLYWHEEL_TICKS_PER_REV;
            angle=MEASURED_LAUNCH_DEG; distanceFt=RELEASE_DISTANCE_FT; feedPower=FEED_POWER;
            maxDrive=MAX_DRIVE_TICKS_PER_SECOND; maxTurret=MAX_TURRET_TICKS_PER_SECOND;
            gate=new ShotGate(settings); batch++;
            configCsv=","+settings.rpmTolerance+","+settings.accelerationLimit+","+settings.stableSeconds
                    +","+settings.feedSeconds
                    +","+settings.recoverySeconds+","+settings.maxLoopSeconds+","+settings.minVoltage
                    +","+maxDrive+","+maxTurret;
            status="Applied; hold RB; wait READY; fresh A press. Label each attempt with D-pad.";
        } catch (IllegalArgumentException ex) { status=ex.getMessage(); }
    }
    @Override public void loop() {
        HardwareMonitor.get().update();
        // Supports pre-existing MANUAL bulk-cache mode without silently reusing encoder samples.
        for (LynxModule hub:hubs) hub.clearBulkCache();
        double now=System.nanoTime()*1e-9, dt=now-lastLoop; lastLoop=now;
        boolean x=gamepad1.x, a=gamepad1.a, y=gamepad1.y;
        boolean up=gamepad1.dpad_up, down=gamepad1.dpad_down, left=gamepad1.dpad_left, right=gamepad1.dpad_right;
        int attempts=gate==null ? 0 : gate.attempts();
        boolean changed=false;
        if (attempts>labeledAttempt && gate!=null && !gate.feed()) {
            if (y&&!lastY) { outcome="INVALID"; changed=true; }
            else if (up&&!lastUp) { outcome="STAY"; stays++; changed=true; }
            else if (down&&!lastDown) { outcome="EXIT"; exits++; changed=true; }
            else if (left&&!lastLeft) { outcome="MISS"; misses++; changed=true; }
            else if (right&&!lastRight) { outcome="RIM"; rims++; changed=true; }
            if (changed) labeledAttempt=attempts;
        }
        if (x&&!lastX && !changed && !gamepad1.right_bumper && attempts==labeledAttempt) {
            // Prior counts remain in CSV; next applied batch uses fresh per-batch attempt IDs.
            apply(); labeledAttempt=0; outcome="NONE"; changed=true;
        }
        double measured=gate==null ? 0 : motor.getVelocity()*60/ticksPerRev;
        double volts=battery.getVoltage(), driveSpeed=0;
        for (DcMotorEx wheel:drive) {
            wheel.setPower(0); double speed=Math.abs(wheel.getVelocity());
            driveSpeed=Double.isFinite(speed) ? Math.max(driveSpeed,speed) : Double.NaN;
        }
        double turretSpeed=Math.abs(turretEncoder.getVelocity());
        boolean armed=gate!=null && HARDWARE_CONFIRMED && gamepad1.right_bumper && !gamepad1.b && rowCount<6000;
        if (!gamepad1.right_bumper || gamepad1.b) { motorFault=""; armStarted=Double.NaN; }
        if (armed && motorFault.isEmpty()) {
            if (!Double.isFinite(armStarted)) armStarted=now;
            if (gate.state()==ShotGate.State.READY || gate.feed() || gate.state()==ShotGate.State.RECOVERING) armStarted=now;
            if (!Double.isFinite(measured) || !between(volts,10.5,14.5)) motorFault="Invalid RPM/voltage";
            else if (measured>5500 || measured< -150) motorFault="Overspeed or encoder sign mismatch";
            else if (now-armStarted>5) motorFault="No stable speed within 5 seconds";
        }
        armed=armed && motorFault.isEmpty();
        if (gate!=null) {
            boolean stationary=Double.isFinite(driveSpeed) && Double.isFinite(turretSpeed)
                    && driveSpeed<=maxDrive && turretSpeed<=maxTurret;
            // Consume A even while another attempt awaits a label; no delayed/held-button shot.
            boolean request=a&&!lastA && !changed && gate.attempts()==labeledAttempt;
            int before=gate.attempts();
            gate.update(now,targetRpm,measured,volts,armed,HARDWARE_CONFIRMED,stationary,request);
            if (gate.attempts()!=before) { outcome="UNLABELED"; changed=true; }
            motor.setPower(controller.update(armed ? targetRpm : 0,measured,volts,dt));
            intake.setPower(gate.feed() ? feedPower : 0);
        } else stopActuators();
        String state=gate==null ? "UNCONFIGURED" : gate.state().name();
        if (rowCount<6000 && (changed || !state.equals(lastState) || now-lastLog>=(gate!=null && (gate.feed() || gate.state()==ShotGate.State.RECOVERING) ? .02 : .1))) {
            log.append(now).append(',').append(dt).append(',').append(batch).append(',').append(gate==null?0:gate.attempts()).append(',').append(state)
                .append(',').append(targetRpm).append(',').append(measured).append(',').append(volts)
                .append(',').append(motor.getPower()).append(',').append(driveSpeed).append(',').append(turretSpeed)
                .append(',').append(angle).append(',').append(distanceFt)
                .append(',').append(ticksPerRev).append(',').append(feedPower).append(',').append(outcome).append(configCsv).append('\n');
            rowCount++; lastLog=now; lastState=state;
        }
        telemetry.addData("Setup",status);
        if (!motorFault.isEmpty()) telemetry.addData("Motor fault; release RB",motorFault);
        telemetry.addData("Gate",state+": "+(gate==null?"Apply setup":gate.reason()));
        telemetry.addData("RPM target / measured","%.0f / %.0f",targetRpm,measured);
        telemetry.addData("Volts / dt","%.2f / %.3f",volts,dt);
        telemetry.addData("Motion drive / turret tps","%.1f / %.1f",driveSpeed,turretSpeed);
        telemetry.addData("Batch attempts / labeled","%d / %d",gate==null?0:gate.attempts(),labeledAttempt);
        telemetry.addData("Stay / exit / miss / rim","%d / %d / %d / %d",stays,exits,misses,rims);
        double speed=PollenPhysics.vacuumSpeed(distanceFt*.3048,angle);
        telemetry.addData("REFERENCE ONLY vacuum m/s / ideal RPM","%.2f / %.0f",speed,PollenPhysics.wheelRpm(speed,.5));
        telemetry.addData("CSV rows (saved on STOP)",rowCount+" / 6000");
        telemetry.addLine("RB arm; A attempt; B cancel. D-pad: UP stay, DOWN exit, LEFT miss, RIGHT rim; Y invalid/no ball.");
        lastX=x; lastA=a; lastY=y; lastUp=up; lastDown=down; lastLeft=left; lastRight=right;
    }
    private void stopActuators() {
        if (intake!=null) intake.setPower(0);
        controller.reset();
        if (motor!=null) motor.setPower(0);
        if (drive!=null) for (DcMotorEx wheel:drive) wheel.setPower(0);
    }
    @Override public void stop() {
        stopActuators(); // Disk work happens only AFTER outputs are stopped, never in the control loop.
        if (logFile==null) return;
        try {
            File parent=logFile.getParentFile();
            if (!parent.isDirectory()&&!parent.mkdirs()) throw new java.io.IOException("Cannot create log directory");
            try (OutputStreamWriter writer=new OutputStreamWriter(new FileOutputStream(logFile),StandardCharsets.UTF_8)) {
                writer.write(log.toString());
            }
        } catch (java.io.IOException ex) {
            com.qualcomm.robotcore.util.RobotLog.ee("PollenShotExperiment",ex,"CSV save failed");
        }
    }
}
