// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.utils.PoseUtils;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.LinearInterpolator;
import frc.utils.PoseEstimatorSubsystem;

public class Shoulder extends ProfiledPIDSubsystem {

  CANSparkMax leftShoulder;
  CANSparkMax rightShoulder;
  SparkPIDController shoulderPID;

  Encoder relative = new Encoder(1, 2);
  DutyCycleEncoder absolute = new DutyCycleEncoder(0);
  // Old Conv Factor 123.71

  ShuffleboardTab shoulderTab = Shuffleboard.getTab("Shoulder");
  GenericEntry shoulderPos;
  GenericEntry shoulderSetpoint;
  GenericEntry shoulderP;
  GenericEntry shoulderI;
  GenericEntry shoulderD;
  GenericEntry shoulderFF;
  GenericEntry busVoltage;
  GenericEntry appliedOutput;

  double shoulderCurSetpoint = ShoulderConstants.shoulderHome;

  PoseEstimatorSubsystem poseEst;

  private LinearInterpolator interpolator = new LinearInterpolator(ShoulderConstants.shoulderData);

  SlewRateLimiter shoulderSlew = new SlewRateLimiter(60);

  ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  Supplier<Boolean> hasNote;

  boolean manualOverride = false;

  AprilTagFieldLayout layout;

  int counter = 0;

  double shoulderOffset = 0;

  /** Creates a new Shoulder. */
  public Shoulder(Supplier<Boolean> m_hasNote, PoseEstimatorSubsystem m_poseEst) {
    super(
        new ProfiledPIDController(
            ShoulderConstants.kShoulderP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ShoulderConstants.kMaxVelocity,
                ShoulderConstants.kMaxAcceleration)),
        ShoulderConstants.shoulderHome);

    poseEst = m_poseEst;
    hasNote = m_hasNote;

    leftShoulder = new CANSparkMax(ShoulderConstants.leftShoulderCanId, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(ShoulderConstants.rightShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    rightShoulder.setSmartCurrentLimit(40, 40);
    leftShoulder.setSmartCurrentLimit(40, 40);

    rightShoulder.follow(leftShoulder, true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);
    
    shoulderSetpoint = shoulderTab.add("ShoulderSetpoint", shoulderCurSetpoint).getEntry();
    shoulderPos = shoulderTab.add("Shoulder Pos", getShoulderPos()).getEntry();

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();

    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

    relative.reset();
    relative.setDistancePerPulse(123.71/2048);
    absolute.setDutyCycleRange(1/1025, 1025/1025);
    absolute.setDistancePerRotation(123.71);
    //absolute.setPositionOffset(15);
    shoulderOffset = ((1 - absolute.getAbsolutePosition()) * 123.71) + 7.195;
    System.out.println(shoulderOffset);

    setGoal(shoulderCurSetpoint);
    m_controller.reset(getShoulderPos());

    appliedOutput = shoulderTab.add("AppliedOutput", leftShoulder.getAppliedOutput()).getEntry();
    busVoltage = shoulderTab.add("BusVoltage", leftShoulder.getBusVoltage()).getEntry();
  }

  public double getShoulderPos() {
    return relative.getDistance() + shoulderOffset;
  }

  public boolean isHome() {
    return ShoulderConstants.shoulderHome - 2 <= getShoulderPos() && getShoulderPos() <= ShoulderConstants.shoulderHome + 2 ;
 }

  public void goHome() {
    shoulderCurSetpoint = ShoulderConstants.shoulderHome;
  }

  public void setShoulderSpeed(double speed) {
    rightShoulder.set(speed);
  }

  public void moveShoulderToPos(double degrees) {
    //shoulderPID.setReference(shoulderSlew.calculate(degrees), ControlType.kPosition);
    //shoulderPID.setReference(degrees, ControlType.kPosition);
    if (degrees > 135) { 
      degrees = 135;
    }
    if (degrees < 15) {
      degrees = 15;
    }
    setGoal(degrees);
  }

  public void setShoulderSetpoint(double setpoint) {
    shoulderCurSetpoint = setpoint;
  }

  public double getShoulderSetpoint() {
    return shoulderCurSetpoint;
  }

  public boolean atShoulderSetpoint() {
    return shoulderCurSetpoint - 1 <= getShoulderPos() && getShoulderPos() <= shoulderCurSetpoint + 1;
  }

  public double calculateAngleFromDistance(double distance) {
    return interpolator.getInterpolatedValue(distance);
  }

  public boolean getManualOverride() {
    return manualOverride;
  }

  public void setManualOverride(boolean override) {
    manualOverride = override;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    moveShoulderToPos(shoulderCurSetpoint);
    useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());

    if (!manualOverride && hasNote.get() && poseEst.getLatestTag().hasTargets() && isSpeakerTag(poseEst.getLatestTag().getBestTarget().getFiducialId())) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(poseEst.getLatestTag().getBestTarget().getPitch()));
      SmartDashboard.putNumber("Vision Range", range);
      if (PoseUtils.inRange(range)) {
        //setShoulderSetpoint(interpolator.getInterpolatedValue(range));
      }
      counter = 0;
    } else if (!manualOverride && hasNote.get()) {
      double distanceToTarget = PhotonUtils.getDistanceToPose(poseEst.getCurrentPose(), layout.getTagPose(PoseUtils.getSpeakerTag()).get().toPose2d());
      distanceToTarget = (1.47 * distanceToTarget) + -1.46;
      SmartDashboard.putNumber("Pose Range", distanceToTarget);
      if (PoseUtils.inRange(distanceToTarget)) {
        //setShoulderSetpoint(interpolator.getInterpolatedValue(distanceToTarget));
      }
      counter = 0;
    } else {
      if (counter == (1 * 50)) {
        //goHome();
      } else {
        counter++;
      }
    }

    shoulderPos.setDouble(getShoulderPos());
    shoulderSetpoint.setDouble(shoulderCurSetpoint);
    busVoltage.setDouble(leftShoulder.getBusVoltage());
    appliedOutput.setDouble(leftShoulder.getAppliedOutput());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      //shoulderSetpoint.setDouble(shoulderCurSetpoint);
      double tempSetpoint = shoulderSetpoint.getDouble(shoulderCurSetpoint);
      if (shoulderCurSetpoint != tempSetpoint) {
        setShoulderSetpoint(tempSetpoint);
      }
    }
  }

  private boolean isSpeakerTag(double tag) {
    return (tag == 7 || tag == 4);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    //System.out.println(output + " " + setpoint.position + " " + setpoint.velocity);
    leftShoulder.set(output + ff);
  }

  @Override
  protected double getMeasurement() {
    return getShoulderPos();
  }
}
