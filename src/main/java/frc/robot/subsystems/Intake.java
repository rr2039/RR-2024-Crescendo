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
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  CANSparkMax belt;
  CANSparkMax flapper;
  CANSparkMax intake;
  AbsoluteEncoder flapperEnc;
  SparkPIDController flapperPID;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private ColorMatch m_colorMatcher = new ColorMatch();

  ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  GenericEntry flapperPos;
  GenericEntry flapperSetpoint;
  GenericEntry flapperP;
  GenericEntry flapperI;
  GenericEntry flapperD;
  GenericEntry flapperFF;
  GenericEntry hasNote;

  /** Creates a new Intake. */
  public Intake() {
    belt = new CANSparkMax(IntakeConstants.beltCanId, MotorType.kBrushless);
    flapper = new CANSparkMax(IntakeConstants.flapperCanId, MotorType.kBrushless);
    intake = new CANSparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);
    intake.setInverted(true);

    flapperEnc = flapper.getAbsoluteEncoder(Type.kDutyCycle);
    flapperEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR
    flapperPos = intakeTab.add("FlapperPos", getFlapperPos()).getEntry();

    flapper.restoreFactoryDefaults();

    flapper.setSoftLimit(SoftLimitDirection.kForward, 0);
    flapper.setSoftLimit(SoftLimitDirection.kReverse, -90);

    flapper.setIdleMode(IdleMode.kBrake);

    flapperPID = flapper.getPIDController();
    flapperPID.setFeedbackDevice(flapperEnc);
    
    flapperPID.setP(IntakeConstants.kFlapperP);
    flapperP = intakeTab.add("FlapperP", flapperPID.getP(0)).getEntry();
    flapperPID.setI(IntakeConstants.kFlapperI);
    flapperI = intakeTab.add("FlapperI", flapperPID.getP(0)).getEntry();
    flapperPID.setD(IntakeConstants.kFlapperD);
    flapperD = intakeTab.add("FlapperD", flapperPID.getP(0)).getEntry();
    flapperPID.setFF(IntakeConstants.kFlapperFF);
    flapperFF = intakeTab.add("FlapperFF", flapperPID.getP(0)).getEntry();

    flapper.burnFlash();

    m_colorMatcher.addColorMatch(IntakeConstants.noteColor);
    hasNote = intakeTab.add("Has Note", hasNote()).getEntry();
  }

  public boolean hasNote() {
    return m_colorMatcher.matchClosestColor(colorSensor.getColor()).color == IntakeConstants.noteColor;
  }

  public double getFlapperPos() {
    return flapperEnc.getPosition();
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
    flapperPos.setDouble(getFlapperPos());
    hasNote.setBoolean(hasNote());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      double tempP = flapperP.getDouble(flapperPID.getP(0));
      if (flapperPID.getP(0) != tempP) {
        flapperPID.setP(tempP, 0);
      }
      double tempI = flapperI.getDouble(flapperPID.getI(0));
      if (flapperPID.getI(0) != tempI) {
        flapperPID.setI(tempI, 0);
      }
      double tempD = flapperD.getDouble(flapperPID.getD(0));
      if (flapperPID.getD(0) != tempD) {
        flapperPID.setD(tempD, 0);
      }
      double tempFF = flapperFF.getDouble(flapperPID.getFF(0));
      if (flapperPID.getFF(0) != tempFF) {
        flapperPID.setFF(tempFF, 0);
      }
    }
  }
}
