// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class ShooterFeed extends Command {
  Intake intake;
  Shooter shooter;
  Shoulder shoulder;
  double counter = 0;

  /** Creates a new ShooterFeed. */
  public ShooterFeed(Intake m_intake, Shooter m_shooter, Shoulder m_shoulder) {
    intake = m_intake;
    shooter = m_shooter;
    shoulder = m_shoulder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atSetpoint() && shoulder.atShoulderSetpoint()) {
      intake.setBeltSpeed(1);
      counter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setBeltSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == (0.5 * 50);
  }
}
