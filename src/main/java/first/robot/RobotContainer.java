// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Command;
// import org.wpilib.command3.Coroutine;
// import org.wpilib.command3.Scheduler;
// import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command3.button.CommandNiDsPS5Controller;
import org.wpilib.driverstation.Gamepad;
// import org.wpilib.framework.OpModeRobot;
// import org.wpilib.framework.OpModeRobot.*;
// import org.wpilib.hardware.hal.RobotMode;
// import org.wpilib.opmode.PeriodicOpMode;

// import first.robot.subsystems.drive.*;
import org.wpilib.tunable.Selectable;
// import org.wpilib.smartdashboard.SmartDashboard;

// import com.pathplanner.lib.auto.AutoBuilder;
// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.geometry.Rotation2d;

import first.robot.commands.DriveCommands;
// import first.robot.generated.TunerConstants;
import first.robot.subsystems.drive.Drive;
// import first.robot.subsystems.drive.ModuleIO;
// import first.robot.subsystems.drive.ModuleIOSim;
// import first.robot.subsystems.drive.ModuleIOTalonFX;

// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import first.robot.opmodes.PrintTeleOpMode;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive;// = new Drive();


  private final Selectable<Command> autoChooser;// = new Selectable<>();

  private final CommandNiDsPS5Controller controller;// = new CommandNiDsPS5Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Drive drive,
  Selectable<Command> autoChooser,
  CommandNiDsPS5Controller controller) {

    this.drive = drive;
    this.autoChooser = autoChooser;
    this.controller = controller;
    // Configure the button bindings
    configureButtonBindings();

    //
    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link org.wpilib.driverstation.GenericHID} or one of its subclasses ({@link
   * org.wpilib.driverstation.Joystick} or {@link Gamepad}), and then passing it to a {@link
   * org.wpilib.command3.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // Grab the hatch when the Circle button is pressed.
    // While holding R1, drive at half speed
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDriveCommand(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));
    drive.setDefaultCommand(DriveCommands.joystickDriveAtAngleSequence(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightY(),
                () -> -controller.getRightX()));
    // controller
    //     .triangle()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngleSequence(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> -controller.getRightY(),
    //             () -> -controller.getRightX()));
    
    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         Command.noRequirements(coro -> robotDrive.setMaxOutput(0.5)).named("Set half speed"))
    //     .onFalse(
    //         Command.noRequirements(coro -> robotDrive.setMaxOutput(1)).named("Set full speed"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}