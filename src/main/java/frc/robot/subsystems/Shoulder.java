// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;

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

  /** Creates a new Shoulder. */
  public Shoulder() {
    leftShoulder = new CANSparkMax(ShoulderConstants.leftShoulderCanId, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(ShoulderConstants.rightShoulderCanId, MotorType.kBrushless);

    shoulderEnc = leftShoulder.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR
    shoulderPos = shoulderTab.add("ShoulderPos", getShoulderPos()).getEntry();

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    rightShoulder.setSoftLimit(SoftLimitDirection.kForward, 0);
    rightShoulder.setSoftLimit(SoftLimitDirection.kReverse, -90);

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

    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
  }

  public double getShoulderPos() {
    return shoulderEnc.getPosition();
  }

  public boolean isHome() {
    //return shoulderEnc.getPosition() == ShoulderConstants.shoulderHome;
    return true;
 }

  public void goHome() {
    shoulderCurSetpoint = ShoulderConstants.shoulderHome;
  }

  public void setShoulderSpeed(double speed) {
    rightShoulder.set(speed);
  }

  public void moveShoulderToPos(double degrees) {
    shoulderPID.setReference(degrees, ControlType.kPosition);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    moveShoulderToPos(shoulderCurSetpoint);

    shoulderPos.setDouble(getShoulderPos());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
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
    }
  }
}
