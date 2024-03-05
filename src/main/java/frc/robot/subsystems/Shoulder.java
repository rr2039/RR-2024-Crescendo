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

import org.photonvision.PhotonUtils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.LinearInterpolator;
import frc.utils.PoseEstimatorSubsystem;

public class Shoulder extends SubsystemBase {

  CANSparkMax leftShoulder;
  CANSparkMax rightShoulder;
  AbsoluteEncoder shoulderEnc;
  SparkPIDController shoulderPID;

  ShuffleboardTab shoulderTab = Shuffleboard.getTab("Shoulder");
  GenericEntry shoulderPos;
  GenericEntry shoulderSetpoint;
  GenericEntry shoulderP;
  GenericEntry shoulderI;
  GenericEntry shoulderD;
  GenericEntry shoulderFF;

  double shoulderCurSetpoint = ShoulderConstants.shoulderHome;

  PoseEstimatorSubsystem poseEst;

  private LinearInterpolator interpolator = new LinearInterpolator(ShoulderConstants.shoulderData);

  SlewRateLimiter shoulderSlew = new SlewRateLimiter(60);

  ArmFeedforward feedforward;

  Supplier<Boolean> hasNote;

  boolean manualOverride = false;

  /** Creates a new Shoulder. */
  public Shoulder(Supplier<Boolean> m_hasNote, PoseEstimatorSubsystem m_poseEst) {
    poseEst = m_poseEst;
    hasNote = m_hasNote;

    feedforward = new ArmFeedforward(0, 0.75, 0, 0);

    leftShoulder = new CANSparkMax(ShoulderConstants.leftShoulderCanId, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(ShoulderConstants.rightShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    shoulderEnc = leftShoulder.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderEnc.setInverted(true);
    shoulderEnc.setPositionConversionFactor(123.71);
    shoulderEnc.setVelocityConversionFactor(1);
    shoulderPos = shoulderTab.add("ShoulderPos", getShoulderPos()).getEntry();

    leftShoulder.setSoftLimit(SoftLimitDirection.kForward, 33);
    leftShoulder.setSoftLimit(SoftLimitDirection.kReverse, 112);

    rightShoulder.follow(leftShoulder, true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);

    shoulderPID = leftShoulder.getPIDController();
    shoulderPID.setFeedbackDevice(shoulderEnc);
    shoulderPID.setP(ShoulderConstants.kShoulderP, 0);
    shoulderP = shoulderTab.add("ShoulderP", shoulderPID.getP(0)).getEntry();
    shoulderPID.setI(ShoulderConstants.kShoulderI, 0);
    shoulderI = shoulderTab.add("ShoulderI", shoulderPID.getI(0)).getEntry();
    shoulderPID.setD(ShoulderConstants.kShoulderD, 0);
    shoulderD = shoulderTab.add("ShoulderD", shoulderPID.getD(0)).getEntry();
    shoulderPID.setFF(ShoulderConstants.kShoulderFF, 0);
    shoulderFF = shoulderTab.add("ShoulderFF", shoulderPID.getFF(0)).getEntry();
    
    shoulderSetpoint = shoulderTab.add("ShoulderSetpoint", shoulderCurSetpoint).getEntry();

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
  }

  public double getShoulderPos() {
    return shoulderEnc.getPosition();
  }

  public boolean isHome() {
    return ShoulderConstants.shoulderHome - 2 <= shoulderEnc.getPosition() && shoulderEnc.getPosition() <= ShoulderConstants.shoulderHome + 1 ;
 }

  public void goHome() {
    shoulderCurSetpoint = ShoulderConstants.shoulderHome;
  }

  public void setShoulderSpeed(double speed) {
    rightShoulder.set(speed);
  }

  public void moveShoulderToPos(double degrees) {
    shoulderPID.setReference(shoulderSlew.calculate(degrees), ControlType.kPosition);
    //shoulderPID.setReference(degrees, ControlType.kPosition);
  }

  public void setShoulderSetpoint(double setpoint) {
    shoulderCurSetpoint = setpoint;
  }

  public double getShoulderSetpoint() {
    return shoulderCurSetpoint;
  }

  public boolean atShoulderSetpoint() {
    return getShoulderPos() == shoulderCurSetpoint;
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

    if (!manualOverride && hasNote.get() && poseEst.getLatestTag().hasTargets() && isSpeakerTag(poseEst.getLatestTag().getBestTarget().getFiducialId())) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(poseEst.getLatestTag().getBestTarget().getPitch()));
      //System.out.println(range);
      //System.out.println(poseEst.getLatestTag().getBestTarget().getYaw());
      if (1 <= range || range <= 5) {
        setShoulderSetpoint(interpolator.getInterpolatedValue(range));
      }
      //setShoulderSetpoint(calculateAngleFromDistance(range));
    } else {
      goHome();
    }

    shoulderPos.setDouble(getShoulderPos());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      shoulderSetpoint.setDouble(shoulderCurSetpoint);
      double tempP = shoulderP.getDouble(shoulderPID.getP(0));
      if (shoulderPID.getP(0) != tempP) {
        shoulderPID.setP(tempP, 0);
      }
      double tempI = shoulderI.getDouble(shoulderPID.getI(0));
      if (shoulderPID.getI(0) != tempI) {
        shoulderPID.setI(tempI, 0);
      }
      double tempD = shoulderD.getDouble(shoulderPID.getD(0));
      if (shoulderPID.getD(0) != tempD) {
        shoulderPID.setD(tempD, 0);
      }
      double tempFF = shoulderFF.getDouble(shoulderPID.getFF(0));
      if (shoulderPID.getFF(0) != tempFF) {
        shoulderPID.setFF(tempFF, 0);
      }
      double tempSetpoint = shoulderSetpoint.getDouble(shoulderCurSetpoint);
      if (shoulderCurSetpoint != tempSetpoint) {
        setShoulderSetpoint(tempSetpoint);
      }
    }
  }

  private boolean isSpeakerTag(double tag) {
    return (tag == 7 || tag == 8 || tag == 3 || tag == 4);
  }
}
