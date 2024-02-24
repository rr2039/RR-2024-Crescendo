// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  CANSparkMax rightShooter;
  CANSparkMax leftShooter;
  AbsoluteEncoder shooterEnc;
  SparkPIDController shooterPID;

  /** Creates a new Shooter. */
  public Shooter() {
    rightShooter = new CANSparkMax(ShooterConstants.rightShooterCanId, MotorType.kBrushless);
    leftShooter = new CANSparkMax(ShooterConstants.leftShooterCanId, MotorType.kBrushless);

    shooterEnc = rightShooter.getAbsoluteEncoder(Type.kDutyCycle);
    shooterEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR

    rightShooter.restoreFactoryDefaults();
    leftShooter.restoreFactoryDefaults();

    leftShooter.follow(rightShooter, true);

    rightShooter.setIdleMode(IdleMode.kBrake);
    leftShooter.setIdleMode(IdleMode.kBrake);

    shooterPID = rightShooter.getPIDController();
    shooterPID.setFeedbackDevice(shooterEnc);

    shooterPID.setP(ShooterConstants.kShooterP);
    shooterPID.setI(ShooterConstants.kShooterI);
    shooterPID.setD(ShooterConstants.kShooterD);
    shooterPID.setFF(ShooterConstants.kShooterFF);

    rightShooter.burnFlash();
    leftShooter.burnFlash();
  }

  public void setShooterSpeed(double speed) {
    rightShooter.set(speed);
  }
  
  public void moveFlapperToPos(double velocity) {
    shooterPID.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
