// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  CANSparkMax leftClimber;
  CANSparkMax rightClimber;

  SparkPIDController climberPID;

  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new CANSparkMax(ClimberConstants.leftClimberCanId, MotorType.kBrushless);
    rightClimber = new CANSparkMax(ClimberConstants.rightClimberCanId, MotorType.kBrushless);

    rightClimber.restoreFactoryDefaults();
    leftClimber.restoreFactoryDefaults();

    rightClimber.setSmartCurrentLimit(30, 40);
    leftClimber.setSmartCurrentLimit(30, 40);

    rightClimber.setSoftLimit(SoftLimitDirection.kForward, 0);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, -90);

    leftClimber.follow(rightClimber, true);

    rightClimber.setIdleMode(IdleMode.kBrake);
    leftClimber.setIdleMode(IdleMode.kBrake);

    climberPID = rightClimber.getPIDController();
    climberPID.setP(ClimberConstants.kClimberP);

    climberPID.setI(ClimberConstants.kClimberI);

    climberPID.setD(ClimberConstants.kClimberD);

    climberPID.setFF(ClimberConstants.kClimberFF);


    rightClimber.burnFlash();
    leftClimber.burnFlash();
  }

  public void setClimberSpeed(double speed) {
    rightClimber.set(speed);
  }

  public void moveClimberToPos(double degrees) {
    climberPID.setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
