// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
// import org.wpilib.hardware.hal.FRCNetComm.tInstances;
// import org.wpilib.hardware.hal.FRCNetComm.tResourceType;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;
// import org.wpilib.util.Alert.*;
// import org.wpilib.util.Alert.AlertType;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command3.*;
// import org.wpilib.wpilibj2.command.sysid.SysIdRoutine;
import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.generated.TunerConstants;
import first.robot.util.LocalADStarAK;
import first.robot.util.TunableDouble;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive implements Mechanism {
  // TunerConstants doesn't include these constants, so they are declared locally
  private static final double DEADBAND = 0.1;
  private static final DoubleSupplier ANGLE_KP = new TunableDouble("Tuning/ANGLE_KP", 5.0);
  private static final DoubleSupplier ANGLE_KD = new TunableDouble("Tuning/ANGLE_KD", 0.4);
  private static final DoubleSupplier ANGLE_MAX_VELOCITY =
      new TunableDouble("Tuning/ANGLE_MAX_VELOCITY", 8.0);
  private static final DoubleSupplier ANGLE_MAX_ACCELERATION =
      new TunableDouble("Tuning/ANGLE_MAX_ACCELERATION", 20.0);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus1.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private static Rotation2d offset = new Rotation2d(0.0); // Offset to apply to gyro angle for odometry
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  
  // private final SysIdRoutine sysId;
  // private final Alert gyroDisconnectedAlert =
  //     new Alert("Disconnected gyro, using kinematics as fallback.", Level.HIGH);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);

  private TunableDouble driveKP =
      new TunableDouble("Drive/drivekP", TunerConstants.BackLeft.DriveMotorGains.kP);
  private TunableDouble driveKV =
      new TunableDouble("Drive/drivekV", TunerConstants.BackLeft.DriveMotorGains.kV);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    driveKP.runOnChange(this::setKPGain);
    driveKV.runOnChange(this::setKVGain);

    // Usage reporting for swerve template
    // HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     this::getChassisVelocities,
    //     this::runVelocity,
    //     new PPHolonomicDriveController(
    //         new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(10, 0.0, 0.6)),
    //     PP_CONFIG,
    //     () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
    //     this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

  //   Configure SysId
  //   sysId =
  //       new SysIdRoutine(
  //           new SysIdRoutine.Config(
  //               null,
  //               null,
  //               null,
  //               (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
  //           new SysIdRoutine.Mechanism(
  //               (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log current commands requiring Drive mechanism
    // clearing the running commands (in case fewer are running now than last loop)
    List<Command> runningCommands = this.getRunningCommands();
    for (int i = 0; i < 4; i++) {
      Logger.recordOutput("Drive/RunningCommand_"+i,"N/A");
    }

    for (Command cmd : runningCommands) {
      Logger.recordOutput("Drive/RunningCommand_"+runningCommands.indexOf(cmd),cmd.name());
    }

    // Log empty setpoint states when disabled
    if (RobotState.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance
                    - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
      // Update gyro angle
      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        // Use the sampled gyro angle when available.
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Fall back to the angle delta from wheel odometry if gyro data is unavailable.
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation.minus(offset), modulePositions);
    }

    // Update gyro alert
    //gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public static void resetOdometry(Drive drive) {
    drive.odometryLock.lock();
    drive.poseEstimator.resetPosition(drive.rawGyroRotation, drive.getModulePositions(), Pose2d.ZERO);
    drive.odometryLock.unlock();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().orElse(new Rotation2d());
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return run(() -> runCharacterization(0.0))
  //       .withTimeout(1.0)
  //       .andThen(sysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  // }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  private ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  // public void addVisionMeasurement(
  //     Pose2d visionRobotPoseMeters,
  //     double timestampSeconds,
  //     Matrix<N3, N1> visionMeasurementStdDevs) {
  //   poseEstimator.addVisionMeasurement(
  //       visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  // }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public void setKPGain(double value) {
    for (int i = 0; i < 4; ++i) {
      modules[i].setKPGain(value);
    }
  }

  public void setKDGain(double value) {
    for (int i = 0; i < 4; ++i) {
      modules[i].setKDGain(value);
    }
  }

  public void setKVGain(double value) {
    for (int i = 0; i < 4; ++i) {
      modules[i].setKVGain(value);
    }
  }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
        .getTranslation();
  }

  public Command resetAngleController(ProfiledPIDController angleController) {
    return this.run(coroutine -> {
        angleController.reset(getRotation().getRadians());
    })
    .named("resetAngleController");
  }

  public Command clearSamples(
    List<Double> velocitySamples,
    List<Double> voltageSamples) {
    return this.run(coroutine -> {
      velocitySamples.clear();
      voltageSamples.clear();
    })
    .named("clearSamples");
  }
  public Command orientModulesForFF(Drive drive) {
    return this.run(
                coroutine -> {
                  while (true) {
                    drive.runCharacterization(0.0);
                    coroutine.yield();
                  }
                })
            .named("orientModulesForFF");
  }

  public Command restartTimer(Timer timer) {
    return this.run(coroutine -> {
        timer.restart();
      }
    ).named("restartTimer");
  }

  // runs until is interrupted, like disabling the robot or overriding the command
  public Command gatherAndCalcFFData(Drive drive, Timer timer, 
    List<Double> velocitySamples,
    List<Double> voltageSamples
  ) {
    return this.run(
                (coroutine) -> {
                  // loop until command is cancelled, then run the code in finally{}
                  try {
                    while (true) {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                      coroutine.yield();
                    }
                  }
                  finally {
                    int n = velocitySamples.size();
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumXY = 0.0;
                    double sumX2 = 0.0;
                    for (int i = 0; i < n; i++) {
                      sumX += velocitySamples.get(i);
                      sumY += voltageSamples.get(i);
                      sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                      sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                    }
                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Drive FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                  }
                })
                .named("gatherFFData");
    }
  
  
}
