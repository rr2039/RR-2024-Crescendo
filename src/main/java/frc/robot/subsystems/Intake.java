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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
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

  private AnalogInput intakePhotoEye;
  Debouncer debounce = new Debouncer(0.1, DebounceType.kBoth);

  ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  GenericEntry flapperPos;
  GenericEntry flapperSetpoint;
  GenericEntry flapperP;
  GenericEntry flapperI;
  GenericEntry flapperD;
  GenericEntry flapperFF;
  GenericEntry hasNote;
  GenericEntry colorSensorRead;

  double flapperCurSetpoint =  IntakeConstants.flapperHome;

  /** Creates a new Intake. */
  public Intake() {
    belt = new CANSparkMax(IntakeConstants.beltCanId, MotorType.kBrushless);
    flapper = new CANSparkMax(IntakeConstants.flapperCanId, MotorType.kBrushless);
    intake = new CANSparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);

    flapper.restoreFactoryDefaults();
    intake.restoreFactoryDefaults();
    belt.restoreFactoryDefaults();

    // This aint working, so screw it.
    //intake.setInverted(true);

    flapperEnc = flapper.getAbsoluteEncoder(Type.kDutyCycle);
    flapperEnc.setPositionConversionFactor(240); //TODO: CALCULATE CONVERSION FACTOR
    flapperPos = intakeTab.add("FlapperPos", getFlapperPos()).getEntry();

    flapper.setSoftLimit(SoftLimitDirection.kForward, 24);
    flapper.setSoftLimit(SoftLimitDirection.kReverse, 60);

    flapper.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);
    belt.setIdleMode(IdleMode.kBrake);

    flapperPID = flapper.getPIDController();
    flapperPID.setFeedbackDevice(flapperEnc);
    
    flapperPID.setP(IntakeConstants.kFlapperP);
    flapperP = intakeTab.add("FlapperP", flapperPID.getP(0)).getEntry();
    flapperPID.setI(IntakeConstants.kFlapperI);
    flapperI = intakeTab.add("FlapperI", flapperPID.getI(0)).getEntry();
    flapperPID.setD(IntakeConstants.kFlapperD);
    flapperD = intakeTab.add("FlapperD", flapperPID.getD(0)).getEntry();
    flapperPID.setFF(IntakeConstants.kFlapperFF);
    flapperFF = intakeTab.add("FlapperFF", flapperPID.getFF(0)).getEntry();

    flapperSetpoint = intakeTab.add("FlapperSetpoint", flapperCurSetpoint).getEntry();

    flapper.burnFlash();
    belt.burnFlash();
    intake.burnFlash();

    intakePhotoEye = new AnalogInput(0); //new DigitalInput(0);

    m_colorMatcher.addColorMatch(IntakeConstants.noteColor);
    m_colorMatcher.addColorMatch(new Color("#7E6619"));
    m_colorMatcher.addColorMatch(new Color("#876116"));
    m_colorMatcher.addColorMatch(new Color("#876017"));
    m_colorMatcher.addColorMatch(new Color("#856117"));
    m_colorMatcher.addColorMatch(new Color("#8F5C13"));
    m_colorMatcher.addColorMatch(new Color("#925913"));
    m_colorMatcher.addColorMatch(new Color("#915A13"));
    m_colorMatcher.addColorMatch(new Color("#786A1C"));
    m_colorMatcher.addColorMatch(new Color("#7F6619"));
    m_colorMatcher.addColorMatch(new Color("#895F16"));
    hasNote = intakeTab.add("Has Note", hasNote()).getEntry();
    //colorSensorRead = intakeTab.add("Color Sensed", colorSensor.getColor().toString()).getEntry();
  }

  public boolean hasNote() {
    /*ColorMatchResult sensed = m_colorMatcher.matchColor(colorSensor.getColor());
    if (sensed != null) {
      return true;
    } else {
      return false;
    }*/
    //return !intakePhotoEye.get();
    return debounce.calculate(intakePhotoEye.getVoltage() < 1);
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
  
  public void setFlapperSetpoint(double degrees) {
    flapperCurSetpoint = degrees;
  }

  public double getFlapperSetpoint() {
    return flapperCurSetpoint;
  }

  public boolean atFlapperSetpoint() {
    return IntakeConstants.flapperGround - 2 <= getFlapperPos() && getFlapperPos() <= IntakeConstants.flapperGround + 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flapperPos.setDouble(getFlapperPos());
    hasNote.setBoolean(hasNote());

    moveFlapperToPos(flapperCurSetpoint);
    
    //colorSensorRead.setString(colorSensor.getColor().toString());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      flapperSetpoint.setDouble(flapperCurSetpoint);
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
      double tempSetpoint = flapperSetpoint.getDouble (flapperCurSetpoint);
      if (flapperCurSetpoint != tempSetpoint) {
        setFlapperSetpoint(tempSetpoint);
      }
    }
  }
}
