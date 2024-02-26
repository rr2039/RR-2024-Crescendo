// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  CANSparkMax rightShooter;
  CANSparkMax leftShooter;
  AbsoluteEncoder shooterEnc;
  SparkPIDController shooterPID;

  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  GenericEntry shooterPos;
  GenericEntry shooterSetpoint;
  GenericEntry shooterP;
  GenericEntry shooterI;
  GenericEntry shooterD;
  GenericEntry shooterFF;

  Supplier<Boolean> hasNote;
  boolean hadNote = false;

  double shooterCurSetpoint = 0;

  /** Creates a new Shooter. */
  public Shooter(Supplier<Boolean> _hasNote) {
    hasNote = _hasNote;

    rightShooter = new CANSparkMax(ShooterConstants.rightShooterCanId, MotorType.kBrushless);
    leftShooter = new CANSparkMax(ShooterConstants.leftShooterCanId, MotorType.kBrushless);

    shooterEnc = rightShooter.getAbsoluteEncoder(Type.kDutyCycle);
    shooterEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR
    shooterPos = shooterTab.add("ShooterPos", getShooterSpeed()).getEntry();

    rightShooter.restoreFactoryDefaults();
    leftShooter.restoreFactoryDefaults();

    leftShooter.follow(rightShooter, true);

    rightShooter.setIdleMode(IdleMode.kBrake);
    leftShooter.setIdleMode(IdleMode.kBrake);

    shooterPID = rightShooter.getPIDController();
    shooterPID.setFeedbackDevice(shooterEnc);

    shooterPID.setP(ShooterConstants.kShooterP);
    shooterP = shooterTab.add("ShooterP", shooterPID.getP(0)).getEntry();
    shooterPID.setI(ShooterConstants.kShooterI);
    shooterI = shooterTab.add("ShooterI", shooterPID.getP(0)).getEntry();
    shooterPID.setD(ShooterConstants.kShooterD);
    shooterD = shooterTab.add("ShooterD", shooterPID.getP(0)).getEntry();
    shooterPID.setFF(ShooterConstants.kShooterFF);
    shooterFF = shooterTab.add("ShooterFF", shooterPID.getP(0)).getEntry();

    rightShooter.burnFlash();
    leftShooter.burnFlash();
  }

  public double getShooterSpeed() {
    return shooterEnc.getVelocity();
  }

  public void setShooter(double speed) {
    rightShooter.set(speed);
  }
  
  public void setShooterSpeed(double velocity) {
    shooterPID.setReference(velocity, ControlType.kVelocity);
  }

  public void setShooterSetpoint(double velocity) {
    shooterCurSetpoint = velocity;
  }

  public double getShooterSetpoint() {
    return shooterCurSetpoint;
  }

  public boolean atSetpoint() {
    return getShooterSpeed() == shooterCurSetpoint;
  }

  public void setIdle() {
    shooterCurSetpoint = ShooterConstants.idleSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*if (hasNote.get() && !hadNote) {
      shooterCurSetpoint = ShooterConstants.idleSpeed;
      hadNote = true;
    } else if (!hasNote.get() && hadNote) {
      hadNote = false;
    }*/
    setShooterSpeed(shooterCurSetpoint);

    shooterPos.setDouble(getShooterSpeed());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
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
    }
  }
}
