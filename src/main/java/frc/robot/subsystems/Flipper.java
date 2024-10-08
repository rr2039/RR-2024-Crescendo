// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//importing libraries
package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlipperConstants;



public class Flipper extends ProfiledPIDSubsystem {

  CANSparkMax flipper; //flipper represents SparkMax
  AbsoluteEncoder flipperEnc; //flipper encoder type
  SparkPIDController flipperPID; //No one is using it!!!

  ShuffleboardTab flipperTab = Shuffleboard.getTab("Flipper");
  GenericEntry flipperPos;
  GenericEntry flipperSetpoint;
  GenericEntry flipperP;
  GenericEntry flipperI;
  GenericEntry flipperD;
  GenericEntry flipperFF;
  GenericEntry busVoltage;
  GenericEntry appliedOutput;

  double flipperCurSetpoint = FlipperConstants.flipperHome; //CHECK absalute encoder

  ArmFeedforward feedforward = new ArmFeedforward(0, 1.256, 0, 0); //kg 1.256

  Shoulder shoulder;

  /** Creates a new Flipper. */
  public Flipper(Shoulder m_shoulder) {
    super(
        new ProfiledPIDController(
            FlipperConstants.kFlipperP,
            0,
            FlipperConstants.kFlipperD,
            new TrapezoidProfile.Constraints(
                FlipperConstants.kMaxVelocity,
                FlipperConstants.kMaxAcceleration)),
        0);

    shoulder = m_shoulder;


    //create flipper
    flipper = new CANSparkMax(FlipperConstants.flipperCanId, MotorType.kBrushless); 

    //reset
    flipper.restoreFactoryDefaults(); 

    //This line manualy sets num of ticks per revolution for relative encoder
    //flipperEnc = flipper.getAlternateEncoder(8192); //Delete, not needed for absalute 
    //flipperEnc.setPositionConversionFactor(360); //TODO: CALCULATE CONVERSION FACTOR //The Reason Why it's here
    flipperEnc = flipper.getAbsoluteEncoder(Type.kDutyCycle);
    flipperPos = flipperTab.add("FlipperPos", getFlipperPos()).getEntry();



    //software limit 
    flipper.setSoftLimit(SoftLimitDirection.kForward, 90);
    flipper.setSoftLimit(SoftLimitDirection.kReverse, 0);

    //current limit
    flipper.setSmartCurrentLimit(40, 40);

    flipper.setIdleMode(IdleMode.kBrake);
    flipperSetpoint = flipperTab.add("FlipperSetpoint", flipperCurSetpoint).getEntry();

    flipper.burnFlash();

    appliedOutput = flipperTab.add("AppliedOutput", flipper.getAppliedOutput()).getEntry();
    busVoltage = flipperTab.add("BusVoltage", flipper.getBusVoltage()).getEntry();
  }

  
  //Simplified functions ready too use
  public double getFlipperPos() {
    return flipperEnc.getPosition();
  }

  public void setFlipperSpeed(double speed) {
    flipper.set(speed);
  }

  public void moveFlipperToPos(double degrees) {
    //flipperPID.setReference(degrees, ControlType.kPosition);
    setGoal(degrees);
  }

  public void setFlipperSetpoint(double degrees) {
    flipperCurSetpoint = degrees;
  }

  public double getFlipperSetpoint() {
    return flipperCurSetpoint;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flipperPos.setDouble(getFlipperPos());
    busVoltage.setDouble(flipper.getBusVoltage());
    appliedOutput.setDouble(flipper.getAppliedOutput());

    moveFlipperToPos(flipperCurSetpoint);
    useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());




    if (Constants.CODEMODE == Constants.MODES.TEST) {
      flipperSetpoint.setDouble(flipperCurSetpoint);
      double tempSetpoint = flipperSetpoint.getDouble (flipperCurSetpoint);
      if (flipperCurSetpoint != tempSetpoint) {
        setFlipperSetpoint(tempSetpoint);
      }
    }
  }




  @Override
  protected void useOutput(double output, State setpoint) {
    double ff = feedforward.calculate(degreesToRadians(-1*setpoint.position + shoulder.getShoulderPos() -85.4), setpoint.velocity);
    flipper.setVoltage(output + ff);
  }

  @Override
  protected double getMeasurement() {
    return getFlipperPos();
  }
}
