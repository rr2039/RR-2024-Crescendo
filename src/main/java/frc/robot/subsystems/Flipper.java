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
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {

  CANSparkMax flipper;
  AbsoluteEncoder flipperEnc;
  SparkPIDController flipperPID;

  /** Creates a new Flipper. */
  public Flipper() {
    flipper = new CANSparkMax(FlipperConstants.flipperCanId, MotorType.kBrushless);
    
    flipperEnc = flipper.getAbsoluteEncoder(Type.kDutyCycle);
    flipperEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR

    flipper.restoreFactoryDefaults();

    flipper.setSoftLimit(SoftLimitDirection.kForward, 0);
    flipper.setSoftLimit(SoftLimitDirection.kReverse, -90);

    flipper.setIdleMode(IdleMode.kBrake);

    flipperPID = flipper.getPIDController();
    flipperPID.setFeedbackDevice(flipperEnc);

    flipperPID.setP(FlipperConstants.kFlipperP);
    flipperPID.setI(FlipperConstants.kFlipperI);
    flipperPID.setD(FlipperConstants.kFlipperD);
    flipperPID.setFF(FlipperConstants.kFlipperFF);

    flipper.burnFlash();
  }

  public void setFlipperSpeed(double speed) {
    flipper.set(speed);
  }

  public void moveFlipperToPos(double degrees) {
    flipperPID.setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
