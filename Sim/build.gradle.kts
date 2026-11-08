// Sim: desktop (plain JVM) module for robot physics simulations, prediction algorithms and the
// robot-profile database that game simulators read. Nothing here runs on the robot.
//
//   ./gradlew :Sim:run --args="help"           list commands
//   ./gradlew :Sim:run --args="sim all --save" run every sim and store the results
//   ./gradlew :Sim:test                        unit tests
//
// See Sim/README.md and Sim/PROFILE.md.

plugins {
    id("org.jetbrains.kotlin.jvm")
    application
}

// Java 8, like TeamCode, so sim code can move to the robot and every IDE in the team reads it.
// No records, var, switch expressions, text blocks, List.of, Path.of, ... (see AGENTS.md).
java {
    sourceCompatibility = JavaVersion.VERSION_1_8
    targetCompatibility = JavaVersion.VERSION_1_8
}

tasks.withType<JavaCompile>().configureEach {
    options.encoding = "UTF-8"
    options.compilerArgs.add("-Xlint:-options") // JDK 21+ warns that source/target 8 is old
}

// Java and Kotlin JVM targets must match or the build fails.
kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_1_8)
    }
}

application {
    mainClass.set("org.athletech.sim.Main")
    applicationName = "sim"
    // sqlite-jdbc loads a native library; JDK 24+ warns about that unless native access is enabled.
    applicationDefaultJvmArgs = listOf("--enable-native-access=ALL-UNNAMED")
}

// Relative paths (data/robot_profile.db, out/) resolve against the Sim folder.
tasks.named<JavaExec>("run") {
    workingDir = projectDir
}

dependencies {
    // SQLite JDBC driver: the robot profile database (data/robot_profile.db).
    implementation("org.xerial:sqlite-jdbc:3.50.3.0")

    // Apache Commons Math: shared with TeamCode for geometry / linear algebra.
    implementation("org.apache.commons:commons-math3:3.6.1")

    testImplementation("junit:junit:4.13.2")
}

tasks.test {
    useJUnit()
    workingDir = projectDir
    jvmArgs("--enable-native-access=ALL-UNNAMED")
}

//the robot's own control code, so sims and the tuning analyzer run the real thing, not a copy that drifts.
//Only pure-Java classes: anything that touches hardware gets a stand-in in src/ftcStubs
val teamCodeSources = tasks.register<Sync>("copyTeamCodeSources") {
    from(rootProject.file("TeamCode/src/main/java")) {
        include("org/firstinspires/ftc/teamcode/Utility/Encoder.java")
        include("org/firstinspires/ftc/teamcode/Utility/VoltageCompensator.java")
        include("org/firstinspires/ftc/teamcode/Utility/MathUtil.java")
        include("org/firstinspires/ftc/teamcode/Utility/InterpolationData.java")
        include("org/firstinspires/ftc/teamcode/Utility/DynamicTrapezoidalSum.java")
        include("org/firstinspires/ftc/teamcode/Utility/Clock.java")
        include("org/firstinspires/ftc/teamcode/Constants/TunedConstants.java")
        include("org/firstinspires/ftc/teamcode/Constants/FlywheelConstants.java")
        include("org/firstinspires/ftc/teamcode/Constants/ShooterConstants.java")
        include("org/firstinspires/ftc/teamcode/Constants/TurretConstants.java")
        include("org/firstinspires/ftc/teamcode/Systems/Flywheel/Flywheel.java")
        include("org/firstinspires/ftc/teamcode/Systems/Flywheel/FlywheelPIDVSCoefficients.java")
        include("org/firstinspires/ftc/teamcode/Systems/Flywheel/FlywheelVelocityModel.java")
        include("org/firstinspires/ftc/teamcode/Systems/Flywheel/ShotSolver.java")
        include("org/firstinspires/ftc/teamcode/Systems/Flywheel/ShotGate.java")
        include("org/firstinspires/ftc/teamcode/Systems/Turret/TurretController.java")
        include("org/firstinspires/ftc/teamcode/Systems/Drivetrain/DriveController.java")
        include("org/firstinspires/ftc/teamcode/Systems/Turret/TurretRoute.java")
        include("org/firstinspires/ftc/teamcode/Systems/Turret/LanyardMap.java")
        include("org/firstinspires/ftc/teamcode/Tuners/TurretLanyardSweep.java")
        include("org/firstinspires/ftc/teamcode/Tuners/TurretAutotuneEngine.java")
        include("org/firstinspires/ftc/teamcode/Tuners/DriveAutotuneEngine.java")
        include("org/firstinspires/ftc/teamcode/Tuners/ShooterHyperTuning/OnlineTuner.java")
        include("org/firstinspires/ftc/teamcode/Tuners/ShooterHyperTuning/FlywheelHyperTuner.java")
        include("org/firstinspires/ftc/teamcode/Tuners/ShooterHyperTuning/TurretHyperTuner.java")
        include("org/firstinspires/ftc/teamcode/Tuners/ShooterHyperTuning/Findings.java")
        include("org/firstinspires/ftc/teamcode/Tuners/FlywheelVoltageFilterFit.java")
    }
    from(file("src/ftcStubs/java"))
    into(layout.buildDirectory.dir("generated/teamcode-src"))
}

sourceSets.named("main") {
    java.srcDir(teamCodeSources)
}

tasks.register<JavaExec>("analyzeTuning") {
    group = "tuning"
    description = "Pulls new TunerTeleOp sessions and Turret / Drivetrain Autotuner results (if a robot is on adb) and writes TunedConstants.java + tuning-data/REPORT.md"
    dependsOn(":TeamCode:pullTuningData")
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("org.athletech.sim.tuning.TuningAnalyzer")
    workingDir = rootDir
    args(
        "tuning-data/sessions",
        "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants/TunedConstants.java",
        "tuning-data/REPORT.md",
        "tuning-data/turret-autotune",
        "tuning-data/drive-autotune"
    )
}
