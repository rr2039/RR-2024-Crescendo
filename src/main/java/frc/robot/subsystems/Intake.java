// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  CANSparkMax belt;
  CANSparkMax flapper;
  CANSparkMax intake;
  AbsoluteEncoder flapperEnc;
  SparkPIDController flapperPID;

  /** Creates a new Intake. */
  public Intake() {
    belt = new CANSparkMax(IntakeConstants.beltCanId, MotorType.kBrushless);
    flapper = new CANSparkMax(IntakeConstants.flapperCanId, MotorType.kBrushless);
    intake = new CANSparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);

    flapperEnc = flapper.getAbsoluteEncoder(Type.kDutyCycle);
    flapperEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR

    flapper.restoreFactoryDefaults();

    flapper.setSoftLimit(SoftLimitDirection.kForward, 0);
    flapper.setSoftLimit(SoftLimitDirection.kReverse, -90);

    flapper.setIdleMode(IdleMode.kBrake);

    flapperPID = flapper.getPIDController();
    flapperPID.setFeedbackDevice(flapperEnc);

    flapperPID.setP(IntakeConstants.kFlapperP);
    flapperPID.setI(IntakeConstants.kFlapperI);
    flapperPID.setD(IntakeConstants.kFlapperD);
    flapperPID.setFF(IntakeConstants.kFlapperFF);

    flapper.burnFlash();
  }

  public void setBeltSpeed(double speed) {
    belt.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }

  public void setFlapperSpeed(double speed) {
    flapper.set(speed);
  }

  public void moveFlapperToPos(double degrees) {
    flapperPID.setReference(degrees, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
