// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.math.util.Units.degreesToRadians;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum MODES {
    TEST, PROD
  }

  public static final MODES CODEMODE = MODES.PROD;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI / 2; //-Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0; //0
    public static final double kBackLeftChassisAngularOffset =  0; //Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; //Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 17;
    public static final int kRearRightDrivingCanId = 19;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearRightTurningCanId = 18;

    public static final boolean kGyroReversed = true;
  }

  public static final class ShooterConstants {
    // SPARK MAX CAN IDs
    public static final int leftShooterCanId = 20;
    public static final int rightShooterCanId = 21;

    public static final double kShooterP = 0.001;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0.0001;
    public static final double kShooterFF = 0;

    public static final double idleSpeed = 500;

    public static final double[][] shooterData = {
      {1.27, 1200},
      {1.98, 1200},
      {2.52, 1400},
      {3.04, 1500},
      {3.5, 1550},
      {4.03, 1600},
      {4.51, 1700},
      {5.01, 1700}
    };

    public static final double[][] shooterData2 = {
      {1.68, 1200},
      {2.41, 1200},
      {2.97, 1400},
      {3.54, 1500},
      {3.81, 1550},
      {3.97, 1600},
      {4.29, 1700},
      {4.71, 1700}
    };
  }

  public static final class ShoulderConstants {
    //SPARK MAX CAN IDs
    public static final int leftShoulderCanId = 22;
    public static final int rightShoulderCanId = 23;

    public static final double shoulderHome = 50;

    public static final double kShoulderP = (0.05*12.0);
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 0;
    public static final double kShoulderFF = 0;

    public static final double[][] shoulderData = {
      {1.27, 59.5},
      {1.98, 50},
      {2.52, 45},
      {3.04, 39.25},
      {3.5, 36.4},
      {4.03, 33.75},
      {4.51, 31.25},
      {5.01, 29.3}
    };

    public static final double[][] shoulderData2 = {
      {1.68, 59.5},
      {2.41, 50},
      {2.97, 45},
      {3.54, 39.25},
      {3.81, 36.4},
      {3.97, 33.75},
      {4.29, 31.25},
      {4.71, 29.3}
    };

    public static double kMaxVelocity = 215;
    public static double kMaxAcceleration = 4000;
  }

  public static final class IntakeConstants {
    //SPARK MAX CAN IDs
    public static final int beltCanId = 24;
    public static final int flapperCanId = 25;
    public static final int intakeCanId = 26;

    public static final double flapperHome = 0;
    public static final double flapperGround = 12;

    public static final Color noteColor = new Color("#8B5D15");

    public static final double kFlapperP = (0.035*12.0);
    public static final double kFlapperI = 0;
    public static final double kFlapperD = 0;
    public static final double kFlapperFF = 0;

    public static double kMaxVelocity = 100;
    public static double kMaxAcceleration = 1000;
  }

  public static final class FlipperConstants {
    //SPARK MAX CAN IDs
    public static final int flipperCanId = 27;

    public static final double kFlipperP = 0; //0.5
    public static final double kFlipperI = 0;
    public static final double kFlipperD = 0;
    public static final double kFlipperFF = 0;

    public static final double flipperHome = 0;

    public static double kMaxVelocity = 10;
    public static double kMaxAcceleration = 10;
  }

  public static final class ClimberConstants {
    //SPARK MAX CAN IDs
    public static final int leftClimberCanId = 28;
    public static final int rightClimberCanId = 29;

    public static final double kClimberP = 0;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double kClimberFF = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true; //If false Asmith NEOS freak out

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction);// / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.02;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDrivingMotorFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.07;
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5800;
  }

  public static final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 0.161925;
    public static final double CAMERA_PITCH_RADIANS = degreesToRadians(37.5);
    public static final double TARGET_HEIGHT_METERS = 1.4351;
    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(-0.32385, 0.1016, 0.161925),
        new Rotation3d(0.0, degreesToRadians(37.5), degreesToRadians(5.0)));

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }
}
