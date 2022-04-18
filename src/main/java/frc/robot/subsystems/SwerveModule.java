package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final CANCoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  
/**
 * <p>Constructor method to create a SwerveModule object. </p>
 * 
 * @param driveMotorId - CAN ID of the drive motor
 * @param turningMotorId - CAN ID of the turning motor
 * @param driveMotorReversed - Whether or not the driving motor is reversed (True/False)
 * @param turningMotorReversed - Whether or not the driving is reversed (True/False)
 * @param absoluteEncoderId - CAN ID of the absolute encoder
 * @param absoluteEncoderOffset - Offset found by manually alligning wheels and recording current angle
 * @param absoluteEncoderReversed - Whether or not the abolsute encoder is reversed
 */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
         int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new CANCoder(absoluteEncoderId);

      driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
      turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turningMotor.setInverted(turningMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = turningMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
      driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
      turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

      turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
      turningPidController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
  }
//Functions to return various values - Hover over each fun\
 /**
 * Get the position of the motor. This returns the native units of 'rotations' by default, and can be changed by a scale factor using setPositionConversionFactor().
 *
 *@return Returns number of rotations of the motor
 * 
 */
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }
 /**
 * Get the position of the motor. This returns the native units of 'rotations' by default, and can be changed by a scale factor using setPositionConversionFactor().
 *
 *@return Returns number of rotations of the motor
 * 
 */
  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }
 /**
 * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be changed by a scale factor using setVelocityConversionFactor().
 *
 *@return Number the RPM of the motor
 * 
 */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }
/**
 * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be changed by a scale factor using setVelocityConversionFactor().
 *
 *@return Number the RPM of the motor
 * 
 */
  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }
/**
 * Does a bunch of math to find the Angle from the Absolute Encoders
 * 
 * @return Returns Absolute Encoder's angle
 */
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getBusVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
/**
 * Set drive motor encoders to 0 and set the turning encoders to the value of the Absolute Encoder
 */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
  }


  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

}
