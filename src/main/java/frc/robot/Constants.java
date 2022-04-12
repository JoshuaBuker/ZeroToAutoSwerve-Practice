package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
    // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23.5);
    //Defines the Kinematics of your robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    //CAN ID of each Motor's Speedcontroller
        public static final int kFrontLeftDriveMotorPort = 27;
        public static final int kBackLeftDriveMotorPort = 25;
        public static final int kFrontRightDriveMotorPort = 21;
        public static final int kBackRightDriveMotorPort = 23;

        public static final int kFrontLeftTurningMotorPort = 28;
        public static final int kBackLeftTurningMotorPort = 29;
        public static final int kFrontRightTurningMotorPort = 22;
        public static final int kBackRightTurningMotorPort = 24;
    //Boolean value regarding whether or not the encoder is reversed for each motor
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;
    //CAN ID of the CANCoder Encoder for each Swerve Module
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 13;
        public static final int kBackRightDriveAbsoluteEncoderPort = 10;
    //Boolean value reagrding whether or not the CANCoder is reversed or not
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    /*
    Okay so Here is the process to get this value
        1. Deploy the code with all of these values set to zero. 
        2. While robot is deployed, but not enabled, turn each wheel so they are facing forward. Some say to have the bevel gear on the left side, but I have yet to find a good reasoning for this practice.
        3. On Smartdashboard, copy the 'Heading' Value for each motor and paste them here for their respective varible. 
        4. Rebuild code and redeploy it to the robot and when you enable the wheels should all snap forward facing the same direction. 
        PS. Always run the motors forward slowly while on its side or elevated to test if the wheels all turn the same direction. If not, either add 180 to the offset value, or go above and reverse the encoder value.
    */
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.0;
    //Max speed of the robot. Not sure if its safe to increase these, but if you are brave... HAVE FUN
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
//Axis IDs and Joystick Port IDs for configuring stuff in Robot Container
    public static final class OIConstants {
        public static final int kRightJoystickPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 2;
    //Deadband value
        public static final double kDeadband = 0.05;
    }
}