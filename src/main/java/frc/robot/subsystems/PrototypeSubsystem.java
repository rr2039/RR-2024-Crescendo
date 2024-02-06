// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PrototypeSubsystem extends SubsystemBase {

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  RelativeEncoder leftEnc;
  RelativeEncoder rigthEnc;
  double lSpeed = 0.0;
  double rSpeed = 0.0;

  /** Creates a new ExampleSubsystem. */
  public PrototypeSubsystem() {
    leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    leftEnc = leftMotor.getEncoder();
    rightMotor = new CANSparkMax(1, MotorType.kBrushless);
    rigthEnc = rightMotor.getEncoder();
  }

  public void setMotorToSpeed() {
    leftMotor.set(lSpeed);
    rightMotor.set(rSpeed);
  }

  public void stopMotors() {
    leftMotor.set(0);
    rightMotor.set(0)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Motor Speed", leftEnc.getVelocity());
    SmartDashboard.putNumber("Right Motor Speed", rigthEnc.getVelocity());
    lSpeed = SmartDashboard.getNumber("L Set Speed", 0.0);
    rSpeed = SmartDashboard.getNumber("R Set Speed", 0.0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
