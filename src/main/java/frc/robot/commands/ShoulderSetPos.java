// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ShoulderSetPos extends Command {
  Shoulder shoulder;
  boolean setpoint;

  /** Creates a new ShoulderSetPos. */
  public ShoulderSetPos(Shoulder m_shoulder, boolean m_setpoint) {
    shoulder = m_shoulder;
    setpoint = m_setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (setpoint) {
      shoulder.setShoulderSetpoint(shoulder.getShoulderSetpoint()+5);
    } else {
      shoulder.setShoulderSetpoint(shoulder.getShoulderSetpoint()-5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
