// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsPS5Controller;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.hal.RobotMode;
// import org.wpilib.smartdashboard.SendableChooser;
// import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.DataLogManager;
import org.wpilib.tunable.Selectable;

import first.robot.generated.TunerConstants;
import first.robot.subsystems.can_range.RangeIO;
import first.robot.subsystems.can_range.RangeIOCanRange;
import first.robot.subsystems.can_range.RangeSubsystem;
// import first.robot.opmodes.PrintTeleOpMode;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
// import first.robot.subsystems.vision.Vision;
// import first.robot.subsystems.vision.VisionConstants;
// import first.robot.subsystems.vision.VisionIO;
// import first.robot.subsystems.vision.VisionIOPhotonVision;
// import first.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private final Drive drive;

  // private final Vision vision;
  
  private final RangeSubsystem range;

  private final Selectable<Command> autoChooser = new Selectable<>();

  private final CommandNiDsPS5Controller controller = new CommandNiDsPS5Controller(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                // new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera2Name, VisionConstants.robotToCamera2));

        range = new RangeSubsystem(new RangeIOCanRange());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                // new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
            // vision =
            // new Vision(
            //     drive::addVisionMeasurement,
            //     new VisionIOPhotonVisionSim(
            //         VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
            //     new VisionIOPhotonVisionSim(
            //         VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
            //     new VisionIOPhotonVisionSim(
            //         VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));
        range = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                // new GyroIO() {},
                new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO[] {});

        range = new RangeSubsystem(new RangeIO() {});
        break;
    }

    robotContainer = new RobotContainer(drive, autoChooser, controller);

    // Start recording to data log
    DataLogManager.start();

    // Record DS control and joystick data.
    // Change to `false` to not record joystick data.
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // this.addOpModeFactory(
    //   () -> new PrintTeleOpMode(),
    //   RobotMode.TELEOPERATED,
    //   "PrintTeleOpMode"
    // );

    //  this.publishOpModes();
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

  }
/**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // vision.periodic();
    // because the canrange has no sim implementation
    if (Constants.currentMode != Constants.simMode) {
      range.periodic();
    }
    // ^^^ update before drive for pose estimation's sake?
    drive.periodic();


    Scheduler.getDefault().run();
    }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */

}
