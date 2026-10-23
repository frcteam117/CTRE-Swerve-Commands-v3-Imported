// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.commands;

import org.wpilib.math.util.*;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.littletonrobotics.junction.Logger;
// import org.wpilib.wpilibj2.command.Command;
// import org.wpilib.wpilibj2.command.Commands;
import org.wpilib.command3.*;
import first.robot.subsystems.drive.Drive;
import first.robot.util.TunableDouble;

import static org.wpilib.units.Units.Seconds;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class DriveCommands {
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


  private DriveCommands() {}

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

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */


  private static Rotation2d rotTarget = Rotation2d.ZERO;

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   * 
   
   */

  public static void joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisVelocities speeds =
              new ChassisVelocities(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              MatchState.getAlliance().isPresent()
                  && MatchState.getAlliance().get() == Alliance.RED;
          drive.runVelocity(
              speeds.toRobotRelative(
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation())
              );

  }

  public static Command joystickDriveAtAngleCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotxSupplier,
      DoubleSupplier rotySupplier,
      ProfiledPIDController angleController) {

    return drive.runRepeatedly( () -> {
    // Create PID controller
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    rotTarget =
        // new Translation2d(rotxSupplier.getAsDouble(), rotySupplier.getAsDouble()).getAngle().orElse(new Rotation2d());
        new Translation2d(0, 0).getAngle().orElse(new Rotation2d());
        // PROBLEM ^^^

              angleController.setP(ANGLE_KP.getAsDouble());
              angleController.setD(ANGLE_KD.getAsDouble());
              angleController.setConstraints(
                  new TrapezoidProfile.Constraints(
                      ANGLE_MAX_VELOCITY.getAsDouble(), ANGLE_MAX_ACCELERATION.getAsDouble()));
                    Logger.recordOutput("rotTarget",rotTarget);
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              Translation2d rotTargetVec =
                  new Translation2d(rotxSupplier.getAsDouble(), rotySupplier.getAsDouble());
              if (rotTargetVec.getNorm() > .15) {
                rotTarget = rotTargetVec.getAngle().orElse(new Rotation2d());
              }
                                  Logger.recordOutput("rotTargetVec",rotTargetVec);


              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotTarget.getRadians());

                    Logger.recordOutput("omega",omega);


              // Convert to field relative speeds & send command
              ChassisVelocities speeds =
                  new ChassisVelocities(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  MatchState.getAlliance().isPresent()
                      && MatchState.getAlliance().get() == Alliance.RED;
              drive.runVelocity(
                  speeds.toRobotRelative(
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            }
        )
        .named("joystickDriveAtAngleCommand");
  }

  public static Command joystickDriveCommand(Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
        // implicitly require `this`
        return drive.runRepeatedly(() -> joystickDrive(drive,xSupplier,ySupplier,omegaSupplier)).named("joystickDriveCommand");
    }
  
  public static Command joystickDriveAtAngleSequence(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotxSupplier,
      DoubleSupplier rotySupplier) {
        
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP.getAsDouble(),
            0.0,
            ANGLE_KD.getAsDouble(),
            new TrapezoidProfile.Constraints(
                ANGLE_MAX_VELOCITY.getAsDouble(), ANGLE_MAX_ACCELERATION.getAsDouble()));
    // Create PID controller
        return 
            drive.resetAngleController(angleController)
            .andThen(joystickDriveAtAngleCommand(drive, xSupplier, ySupplier, rotxSupplier, rotySupplier, angleController))
            .named("joystickDriveAtAngleSequence");

        // Reset PID controller when command starts
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */

  public static Command feedforwardCharacterizationSequence(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    Command timedOrient = drive.orientModulesForFF(drive).withTimeout(Seconds.of(FF_START_DELAY));

    return drive.clearSamples(velocitySamples, voltageSamples)
        .andThen(timedOrient)
        .andThen(drive.restartTimer(timer))
        .andThen(drive.gatherAndCalcFFData(drive, timer, velocitySamples, voltageSamples))
        .named("feedforwardCharacterizationSequence");
  }

//   /** Measures the robot's wheel radius by spinning in a circle. */
//   public static Command wheelRadiusCharacterization(Drive drive) {
//     SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
//     WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

//     return Commands.parallel(
//         // Drive control sequence
//         Commands.sequence(
//             // Reset acceleration limiter
//             Commands.runOnce(
//                 () -> {
//                   limiter.reset(0.0);
//                 }),

//             // Turn in place, accelerating up to full speed
//             Commands.run(
//                 () -> {
//                   double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
//                   drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
//                 },
//                 drive)),

//         // Measurement sequence
//         Commands.sequence(
//             // Wait for modules to fully orient before starting measurement
//             Commands.waitSeconds(1.0),

//             // Record starting measurement
//             Commands.runOnce(
//                 () -> {
//                   state.positions = drive.getWheelRadiusCharacterizationPositions();
//                   state.lastAngle = drive.getRotation();
//                   state.gyroDelta = 0.0;
//                 }),

//             // Update gyro delta
//             Commands.run(
//                     () -> {
//                       var rotation = drive.getRotation();
//                       state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
//                       state.lastAngle = rotation;
//                     })

//                 // When cancelled, calculate and print results
//                 .finallyDo(
//                     () -> {
//                       double[] positions = drive.getWheelRadiusCharacterizationPositions();
//                       double wheelDelta = 0.0;
//                       for (int i = 0; i < 4; i++) {
//                         wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
//                       }
//                       double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

//                       NumberFormat formatter = new DecimalFormat("#0.000");
//                       System.out.println(
//                           "********** Wheel Radius Characterization Results **********");
//                       System.out.println(
//                           "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
//                       System.out.println(
//                           "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
//                       System.out.println(
//                           "\tWheel Radius: "
//                               + formatter.format(wheelRadius)
//                               + " meters, "
//                               + formatter.format(Units.metersToInches(wheelRadius))
//                               + " inches");
//                     })));
//   }

//   private static class WheelRadiusCharacterizationState {
//     double[] positions = new double[4];
//     Rotation2d lastAngle = Rotation2d.ZERO;
//     double gyroDelta = 0.0;
//   }
}
