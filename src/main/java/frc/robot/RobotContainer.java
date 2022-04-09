package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
      swerveSubsystem, 
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));


    configureButtonBindings();
  }

//Function to create buttons and assign functions
  private void configureButtonBindings() {
  //This syntax is to create an "instant command" bound to the ID 2 Button on driverJoystick
    new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }


  public Command getAutonomousCommand() {
/*
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
