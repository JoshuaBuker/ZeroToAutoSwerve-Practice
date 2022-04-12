package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
//Creates Swerve Subsystem 
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
//Creates Joystick object
  private final Joystick rightJoy = new Joystick(OIConstants.kRightJoystickPort);
  

  public RobotContainer() {
//Sets the default Command for the swerveSubsystem. 
//Default Commands are the command that will run when nothing else is running. 
  //Help with syntax here. () -> is a lambda function that runs functions no questions asked, and is needed since we are using Double Suppliers
  // The '-' at the front of the first driverJoystick is simply a negative because the Y axis on joysticks is inverted for flight sim games. So the negative here reverts it back.
  // The '!'' at the front of the last paremter is a 'boolean not'. Since a get raw button will only return true when the button is pressed, added the ! at the front reverses it so it alwasy returns true unless button is pressed.
    //Since we want that value to always be true to be able to use Field Oriented driving, the ! is added. When that button is pressed, the robot will go into Robot Oriented driving.
    swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
      swerveSubsystem, 
      () -> -rightJoy.getRawAxis(OIConstants.kDriverYAxis), 
      () -> rightJoy.getRawAxis(OIConstants.kDriverXAxis), 
      () -> rightJoy.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> !rightJoy.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

//Runs the configureBindings function
    configureButtonBindings();
  }

/**
 * This function runs the methods below to create and assign commands to each created button
 */
  private void configureButtonBindings() {
  //This syntax is to create an "instant command" bound to the ID 2 Button on driverJoystick
    new JoystickButton(rightJoy, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  //For Future commands, create the button object and assign a function rather than doing ^ because 
  }

/**
 * Function to return the desired Autonomous command
 * @return Whatever command gets returned is what will happen during Autonomous Period
 */
  public Command getAutonomousCommand() {
/*This is the trajectory based Auto command created by Zero To Autonomous in his video. However due to lack of knowledge in what this actually does, I will comment it out for now.
    //Create Trajectory Settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond, 
          AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

    //Generate Trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                   new Pose2d(0, 0, new Rotation2d(0)), 
                   List.of(
                            new Translation2d(1, 0),
                            new Translation2d(1, -1)
                   ),
                   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                   trajectoryConfig);

    //Define PID Controlllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //Construct Command to Follow Trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController, 
      swerveSubsystem::setModuleStates, 
      swerveSubsystem);
    
    //Add some initalization and wrap-up, then return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
      );
  }

  */
    return null;
  }
}
