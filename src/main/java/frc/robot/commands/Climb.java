// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shoulder;

public class Climb extends Command {
  Climber climber;
  Shoulder shoulder;
  boolean lift;
  /** Creates a new Climb. */
  public Climb(Climber m_climber, Shoulder m_shoulder, boolean m_lift) {
    climber = m_climber;
    shoulder = m_shoulder;
    lift = m_lift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lift) {
      // Down
      climber.setClimberSpeed(0.5);
      shoulder.setShoulderSetpoint(35);
    } else {
      // Up
      climber.setClimberSpeed(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
