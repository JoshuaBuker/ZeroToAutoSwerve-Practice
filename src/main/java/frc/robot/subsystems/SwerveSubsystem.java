// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
//Creates swerve modules using the Swervemodule class as a blueprint. 
  private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
  //Creates Navx gyroscope object
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
  //Creates SwerveDriveOdometry for use in Autonomous trajectory
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,new Rotation2d(0));
  //Puts initial zero heading command on a seperate thread and delays it by 1 second. Since the Navx will be calibrating when first turned on, it cannot immediately run the zeroHeading command, so it it placed
    //On a seperate Thread and delayed so it will not overlap executes.
    public SwerveSubsystem() {
      new Thread(() -> {
          try {
              Thread.sleep(1000);
              zeroHeading();
          } catch (Exception e) {
          }
      }).start();
  }
/**
 * Reset the Yaw gyro. Resets the Gyro Z (Yaw) axis to a heading of zero. This can be used if there is significant drift in the gyro and it needs to be recalibrated after it has been running.
 */
  public void zeroHeading() {
    gyro.reset();
  }
/**
 * The angle recorded by the Gyroscope is continous, so it will go beyond 360. To account for this, using the remainder function, it divides the gyro value by 360 and returns the remainder.
 * @return Returns the current heading, or rotation of the robot
 */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
/**
 * By getting the robots current angle, it feeds it into a Rotation2d function '.fromDegrees' to convert a degree value to a radian Rotation2d value.
 * @return Returns A Rotation2d value 
 */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
/**
* Returns the position of the robot on the field.
*
*@return The pose of the robot (x and y are in meters).
 */
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }
/**
 * Resets the robot's position on the field.
 * The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
 * @param pose
 * 
 */
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
  }

  @Override
  public void periodic() {

    odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

}