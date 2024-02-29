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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {

  CANSparkMax flipper;
  AbsoluteEncoder flipperEnc;
  SparkPIDController flipperPID;

  ShuffleboardTab flipperTab = Shuffleboard.getTab("Flipper");
  GenericEntry flipperPos;
  GenericEntry flipperSetpoint;
  GenericEntry flipperP;
  GenericEntry flipperI;
  GenericEntry flipperD;
  GenericEntry flipperFF;

  double flipperCurSetpoint = FlipperConstants.flipperHome;

  /** Creates a new Flipper. */
  public Flipper() {
    flipper = new CANSparkMax(FlipperConstants.flipperCanId, MotorType.kBrushless);
    
    flipperEnc = flipper.getAbsoluteEncoder(Type.kDutyCycle);
    flipperEnc.setPositionConversionFactor(2.6); //TODO: CALCULATE CONVERSION FACTOR
    flipperPos = flipperTab.add("FlipperPos", getFlipperPos()).getEntry();

    flipper.restoreFactoryDefaults();

    flipper.setSoftLimit(SoftLimitDirection.kForward, 0);
    flipper.setSoftLimit(SoftLimitDirection.kReverse, -90);

    flipper.setIdleMode(IdleMode.kBrake);

    flipperPID = flipper.getPIDController();
    flipperPID.setFeedbackDevice(flipperEnc);

    flipperPID.setP(FlipperConstants.kFlipperP);
    flipperP = flipperTab.add("FlipperP", flipperPID.getP(0)).getEntry();
    flipperPID.setI(FlipperConstants.kFlipperI);
    flipperI = flipperTab.add("FlipperI", flipperPID.getP(0)).getEntry();
    flipperPID.setD(FlipperConstants.kFlipperD);
    flipperD = flipperTab.add("FlipperD", flipperPID.getP(0)).getEntry();
    flipperPID.setFF(FlipperConstants.kFlipperFF);
    flipperFF = flipperTab.add("FlipperFF", flipperPID.getP(0)).getEntry();

    flipperSetpoint = flipperTab.add("FlipperSetpoint", flipperCurSetpoint).getEntry();

    flipper.burnFlash();
  }

  public double getFlipperPos() {
    return flipperEnc.getPosition();
  }

  public void setFlipperSpeed(double speed) {
    flipper.set(speed);
  }

  public void moveFlipperToPos(double degrees) {
    flipperPID.setReference(degrees, ControlType.kPosition);
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
    if (Constants.CODEMODE == Constants.MODES.TEST) {
      flipperSetpoint.setDouble(flipperCurSetpoint);
      double tempP = flipperP.getDouble(flipperPID.getP(0));
      if (flipperPID.getP(0) != tempP) {
        flipperPID.setP(tempP, 0);
      }
      double tempI = flipperI.getDouble(flipperPID.getI(0));
      if (flipperPID.getI(0) != tempI) {
        flipperPID.setI(tempI, 0);
      }
      double tempD = flipperD.getDouble(flipperPID.getD(0));
      if (flipperPID.getD(0) != tempD) {
        flipperPID.setD(tempD, 0);
      }
      double tempFF = flipperFF.getDouble(flipperPID.getFF(0));
      if (flipperPID.getFF(0) != tempFF) {
        flipperPID.setFF(tempFF, 0);
      }
      double tempSetpoint = flipperSetpoint.getDouble (flipperCurSetpoint);
      if (flipperCurSetpoint != tempSetpoint) {
        setFlipperSetpoint(tempSetpoint);
      }
    }
  }
}
