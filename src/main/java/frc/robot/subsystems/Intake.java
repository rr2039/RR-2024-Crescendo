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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.utils.LEDUtility;
import frc.utils.LEDEffects.LEDEffect;

public class Intake extends ProfiledPIDSubsystem {

  CANSparkMax belt;
  CANSparkMax flapper;
  CANSparkMax intake;

  Encoder flapperRel = new Encoder(4, 5);
  DutyCycleEncoder flapperAbs = new DutyCycleEncoder(3);
  // 240 Old Value Conversion

  DigitalInput queuePhotoEye = new DigitalInput(7);
  DigitalInput intakePhotoEye = new DigitalInput(6);

  Debouncer debounce = new Debouncer(0.1, DebounceType.kBoth);

  ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  GenericEntry flapperPos;
  GenericEntry flapperSetpoint;
  GenericEntry flapperP;
  GenericEntry flapperI;
  GenericEntry flapperD;
  GenericEntry flapperFF;
  GenericEntry hasNote;
  GenericEntry intakingNote;
  GenericEntry colorSensorRead;
  GenericEntry busVoltage;
  GenericEntry appliedOutput;

  double flapperCurSetpoint =  IntakeConstants.flapperHome;

  double flapperOffset = 0;

  LEDUtility ledUtil;

  /** Creates a new Intake. */
  public Intake(LEDUtility m_ledUtil) {
    super(
        new ProfiledPIDController(
            IntakeConstants.kFlapperP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                IntakeConstants.kMaxVelocity,
                IntakeConstants.kMaxAcceleration)),
        IntakeConstants.flapperHome);

    ledUtil = m_ledUtil;

    belt = new CANSparkMax(IntakeConstants.beltCanId, MotorType.kBrushless);
    flapper = new CANSparkMax(IntakeConstants.flapperCanId, MotorType.kBrushless);
    intake = new CANSparkMax(IntakeConstants.intakeCanId, MotorType.kBrushless);

    flapper.restoreFactoryDefaults();
    intake.restoreFactoryDefaults();
    belt.restoreFactoryDefaults();

    // This aint working, so screw it.
    //intake.setInverted(true);

    //belt.setSmartCurrentLimit(40, 40);
    flapper.setSmartCurrentLimit(40, 40);
    //intake.setSmartCurrentLimit(40, 40);
    flapperPos = intakeTab.add("FlapperPos", getFlapperPos()).getEntry();

    flapper.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);
    belt.setIdleMode(IdleMode.kBrake);

    flapperSetpoint = intakeTab.add("FlapperSetpoint", flapperCurSetpoint).getEntry();

    flapper.burnFlash();
    belt.burnFlash();
    intake.burnFlash();

    hasNote = intakeTab.add("Has Note", hasNote()).getEntry();
    intakingNote = intakeTab.add("Intaking Note", intakingNote()).getEntry();
    //colorSensorRead = intakeTab.add("Color Sensed", colorSensor.getColor().toString()).getEntry();

    //flapperRel.reset();
    flapperRel.setDistancePerPulse(240.0/2048.0);
    flapperRel.setReverseDirection(true);
    flapperAbs.setDutyCycleRange(1/1025, 1025/1025);
    flapperAbs.setDistancePerRotation(240);
    //absolute.setPositionOffset(15);
    flapperOffset = ((flapperAbs.getAbsolutePosition()) * 240) - 83;
    System.out.println(flapperOffset);

    m_controller.reset(getFlapperPos());

    appliedOutput = intakeTab.add("AppliedOutput", flapper.getAppliedOutput()).getEntry();
    busVoltage = intakeTab.add("BusVoltage", flapper.getBusVoltage()).getEntry();
  }

  public boolean hasNote() {
    //return !intakePhotoEye.get();
    //return debounce.calculate(intakePhotoEye.getVoltage() < 1);
    return !queuePhotoEye.get();
  }

  public boolean intakingNote() {
    return intakePhotoEye.get();
  }

  public double getFlapperPos() {
    //return flapperEnc.getPosition();
    return flapperRel.getDistance() + flapperOffset;
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
    //flapperPID.setReference(degrees, ControlType.kPosition);
    if (degrees < IntakeConstants.flapperHome) {
      degrees = IntakeConstants.flapperHome;
    }
    if (degrees > IntakeConstants.flapperGround) {
      degrees = IntakeConstants.flapperGround;
    }
    setGoal(degrees);
  }
  
  public void setFlapperSetpoint(double degrees) {
    flapperCurSetpoint = degrees;
  }

  public double getFlapperSetpoint() {
    return flapperCurSetpoint;
  }

  public boolean atGround() {
    return IntakeConstants.flapperGround - 2 <= getFlapperPos() && getFlapperPos() <= IntakeConstants.flapperGround + 2;
  }

  public void goToGround() {
    flapperCurSetpoint = IntakeConstants.flapperGround;
  }

  public void goHome() {
    flapperCurSetpoint = IntakeConstants.flapperHome;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flapperPos.setDouble(getFlapperPos());
    hasNote.setBoolean(hasNote());
    intakingNote.setBoolean(intakingNote());
    flapperSetpoint.setDouble(flapperCurSetpoint);
    busVoltage.setDouble(flapper.getBusVoltage());
    appliedOutput.setDouble(flapper.getAppliedOutput());


    moveFlapperToPos(flapperCurSetpoint);
    useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    if (hasNote()) {
      ledUtil.getStrip(1).setEffect(LEDEffect.SOLID);
      ledUtil.getStrip(1).setColor(Color.kOrange);
      ledUtil.getStrip(3).setEffect(LEDEffect.SOLID);
      ledUtil.getStrip(3).setColor(Color.kOrange);
    } else {
      ledUtil.getStrip(1).setEffect(LEDEffect.SOLID);
      ledUtil.getStrip(1).setColor(Color.kBlack);
      ledUtil.getStrip(3).setEffect(LEDEffect.SOLID);
      ledUtil.getStrip(3).setColor(Color.kBlack);
    }
    
    //colorSensorRead.setString(colorSensor.getColor().toString());
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      flapperSetpoint.setDouble(flapperCurSetpoint);
      double tempSetpoint = flapperSetpoint.getDouble (flapperCurSetpoint);
      if (flapperCurSetpoint != tempSetpoint) {
        setFlapperSetpoint(tempSetpoint);
      }
    }
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    flapper.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return getFlapperPos();
  }
}
