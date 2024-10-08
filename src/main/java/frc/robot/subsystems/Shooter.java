// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.utils.PoseUtils;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.LEDEffects;
import frc.utils.LEDUtility;
import frc.utils.LinearInterpolator;
import frc.utils.PoseEstimatorSubsystem;

public class Shooter extends SubsystemBase {
  
  CANSparkMax rightShooter;
  CANSparkMax leftShooter;
  RelativeEncoder shooterEnc;
  SparkPIDController shooterPID;

  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  GenericEntry shooterPos;
  GenericEntry shooterSetpoint;
  GenericEntry shooterP;
  GenericEntry shooterI;
  GenericEntry shooterD;
  GenericEntry shooterFF;
  GenericEntry busVoltage;
  GenericEntry appliedOutput;

  Supplier<Boolean> hasNote;

  PoseEstimatorSubsystem poseEst;

  double shooterCurSetpoint = 0;
  
  SlewRateLimiter shooterSlew = new SlewRateLimiter(6000);

  private LinearInterpolator interpolator = new LinearInterpolator(ShooterConstants.shooterData);
  private LinearInterpolator interpolator2 = new LinearInterpolator(ShooterConstants.shooterData2);

  SimpleMotorFeedforward arbFF = new SimpleMotorFeedforward(-0.2573, 0.002508);

  boolean manualOverride = false;

  AprilTagFieldLayout layout;

  int counter = 0;

  LEDUtility ledUtil;

  /** Creates a new Shooter. */
  public Shooter(Supplier<Boolean> m_hasNote, PoseEstimatorSubsystem m_poseEst, LEDUtility m_ledUtil) {
    ledUtil = m_ledUtil;
    hasNote = m_hasNote;
    poseEst = m_poseEst;

    rightShooter = new CANSparkMax(ShooterConstants.rightShooterCanId, MotorType.kBrushless);
    leftShooter = new CANSparkMax(ShooterConstants.leftShooterCanId, MotorType.kBrushless);

    rightShooter.restoreFactoryDefaults();
    leftShooter.restoreFactoryDefaults();

    leftShooter.setSmartCurrentLimit(40, 40);
    rightShooter.setSmartCurrentLimit(40, 40);
    
    shooterEnc = rightShooter.getEncoder();
    shooterEnc.setPositionConversionFactor(1); //TODO: CALCULATE CONVERSION FACTOR
    shooterEnc.setVelocityConversionFactor(1);
    shooterPos = shooterTab.add("ShooterPos", getShooterSpeed()).getEntry();

    leftShooter.follow(rightShooter, true);

    rightShooter.setIdleMode(IdleMode.kCoast);
    leftShooter.setIdleMode(IdleMode.kCoast);

    shooterPID = rightShooter.getPIDController();
    shooterPID.setFeedbackDevice(shooterEnc);

    shooterPID.setP(ShooterConstants.kShooterP);
    shooterP = shooterTab.add("ShooterP", shooterPID.getP(0)).getEntry();
    shooterPID.setI(ShooterConstants.kShooterI);
    shooterI = shooterTab.add("ShooterI", shooterPID.getI(0)).getEntry();
    shooterPID.setD(ShooterConstants.kShooterD);
    shooterD = shooterTab.add("ShooterD", shooterPID.getD(0)).getEntry();
    shooterPID.setFF(ShooterConstants.kShooterFF);
    shooterFF = shooterTab.add("ShooterFF", shooterPID.getFF(0)).getEntry();

    shooterSetpoint = shooterTab.add("ShooterSetpoint", shooterCurSetpoint).getEntry();

    appliedOutput = shooterTab.add("AppliedOutput", leftShooter.getAppliedOutput()).getEntry();
    busVoltage = shooterTab.add("BusVoltage", leftShooter.getBusVoltage()).getEntry();

    rightShooter.burnFlash();
    leftShooter.burnFlash();

    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
  }

  public double getShooterSpeed() {
    return shooterEnc.getVelocity();
  }

  public void setShooter(double speed) {
    rightShooter.set(speed);
  }
  
  public void setShooterSpeed(double velocity) {
    shooterPID.setReference(shooterSlew.calculate(velocity), ControlType.kVelocity, 0, arbFF.calculate(velocity));
  }

  public void setShooterSetpoint(double velocity) {
    shooterCurSetpoint = velocity;
  }

  public double getShooterSetpoint() {
    return shooterCurSetpoint;
  }

  public boolean atSetpoint() {
    return (shooterCurSetpoint - 50 <= getShooterSpeed() && getShooterSpeed() <= shooterCurSetpoint + 50) && shooterCurSetpoint > 0;
  }

  public void setIdle() {
    shooterCurSetpoint = ShooterConstants.idleSpeed;
  }

  public double calculateSpeedFromDistance(double distance) {
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
    setShooterSpeed(shooterCurSetpoint);

    PhotonPipelineResult tag = poseEst.getLatestTag();
    if (!manualOverride && hasNote.get() && tag != null && tag.hasTargets() && tag.getLatencyMillis() < 20 && isSpeakerTag(tag.getBestTarget().getFiducialId())) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(tag.getBestTarget().getPitch()));
      //System.out.println("Vision Range: " + range);
      if (PoseUtils.inRange(range)) {
        setShooterSetpoint(interpolator.getInterpolatedValue(range));
      }
      counter = 0;
      //setShoulderSetpoint(calculateAngleFromDistance(range));
    } else if (!manualOverride && hasNote.get()) {
      double distanceToTarget = PhotonUtils.getDistanceToPose(poseEst.getCurrentPose(), layout.getTagPose(PoseUtils.getSpeakerTag()).get().toPose2d());
      //distanceToTarget = (1.25 * distanceToTarget) -1.05;
      //System.out.println("Pose Range: " + distanceToTarget);
      if (PoseUtils.inRange(distanceToTarget)) {
        setShooterSetpoint(interpolator2.getInterpolatedValue(distanceToTarget));
      }
      counter = 0;
    } else {
      if (counter == (1 * 50)) {
        setShooterSetpoint(0);
      } else {
        counter++;
      }
    }

    shooterPos.setDouble(getShooterSpeed());
    busVoltage.setDouble(leftShooter.getBusVoltage());
    appliedOutput.setDouble(leftShooter.getAppliedOutput());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      shooterSetpoint.setDouble(shooterCurSetpoint);
      double tempP = shooterP.getDouble(shooterPID.getP(0));
      if (shooterPID.getP(0) != tempP) {
        shooterPID.setP(tempP, 0);
      }
      double tempI = shooterI.getDouble(shooterPID.getI(0));
      if (shooterPID.getI(0) != tempI) {
        shooterPID.setI(tempI, 0);
      }
      double tempD = shooterD.getDouble(shooterPID.getD(0));
      if (shooterPID.getD(0) != tempD) {
        shooterPID.setD(tempD, 0);
      }
      double tempFF = shooterFF.getDouble(shooterPID.getFF(0));
      if (shooterPID.getFF(0) != tempFF) {
        shooterPID.setFF(tempFF, 0);
      }
      double tempSetpoint = shooterSetpoint.getDouble(shooterCurSetpoint);
      if (shooterCurSetpoint != tempSetpoint) {
        setShooterSetpoint(tempSetpoint);
      }
    }
  }

  private boolean isSpeakerTag(double tag) {
    return (tag == 7 || tag == 4);
  }
}
