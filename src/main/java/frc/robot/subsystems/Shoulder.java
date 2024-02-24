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
import frc.robot.Constants.ShoulderConstants;

public class Shoulder extends SubsystemBase {

  CANSparkMax leftShoulder;
  CANSparkMax rightShoulder;

  SparkPIDController shoulderPID;  

  /** Creates a new Shoulder. */
  public Shoulder() {
    leftShoulder = new CANSparkMax(ShoulderConstants.leftShoulderCanId, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(ShoulderConstants.rightShoulderCanId, MotorType.kBrushless);

    rightShoulder.restoreFactoryDefaults();
    leftShoulder.restoreFactoryDefaults();

    rightShoulder.setSoftLimit(SoftLimitDirection.kForward, 0);
    rightShoulder.setSoftLimit(SoftLimitDirection.kReverse, -90);

    leftShoulder.follow(rightShoulder, true);

    rightShoulder.setIdleMode(IdleMode.kBrake);
    leftShoulder.setIdleMode(IdleMode.kBrake);

    shoulderPID = rightShoulder.getPIDController();
    shoulderPID.setP(ShoulderConstants.kShoulderP);

    shoulderPID.setI(ShoulderConstants.kShoulderI);

    shoulderPID.setD(ShoulderConstants.kShoulderD);

    shoulderPID.setFF(ShoulderConstants.kShoulderFF);


    rightShoulder.burnFlash();
    leftShoulder.burnFlash();
  }

  public void setShoulderSpeed(double speed) {
    rightShoulder.set(speed);
  }

  public void moveShoulderToPos(double degrees) {
    shoulderPID.setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
