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

 /**
   * Sets the NEO motors of the left side of the robot to the specified speed
   * 
   * <p>
   * To drive the left side foward, input a positive speed. For example,
   * {@code moveLeftSide(0.5);}
   * </p>
   * <p>
   * To drive the left side backward, input a negative speed. For example,
   * {@code moveLeftSide(-0.5);}
   * </p>
   * 
   * @param speed - the speed to set the motors to. Value should be between -1.0 and 1.0
   * 
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
