// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends CommandBase {
  
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
  private final BooleanSupplier fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

/**
 * <p> This is the Default Drive Command for teleop. it takes in the x, y, and z axis values and
 * turns it into a chassisSpeed to control the robot. </p>
 * 
 * @param Subsystem - The Swervedrive DrivetrainSubsystem
 * @param xSpdFunction - X Axis Value
 * @param ySpdFunction - Y Axis Value
 * @param turningSpdFunction - Z Axis Value (Rotational)
 * @param fieldOrientedFunction - Whether or not fieldOriented is enabled or not (true/false)
 */
  public DefaultDriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpdFunction, 
          DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
          BooleanSupplier fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //Get Real-time Joystick Inputs
    double xSpeed = xSpdFunction.getAsDouble();
    double ySpeed = ySpdFunction.getAsDouble();
    double turningSpeed = turningSpdFunction.getAsDouble();

    //Apply Deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;

    //Make the Driving Smoother - AKA Slew Rate Limiters
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    //Construct Desired Chassis Speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.getAsBoolean()) {
      //Relative to the field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      //Relative to Robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    //Convert Chassis Speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
