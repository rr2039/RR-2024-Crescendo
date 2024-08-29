// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class FlipperSetPos extends Command {
  Flipper flipper;
  Shooter shooter;
  Shoulder shoulder;

  /** Creates a new FlipperSetPos. */
  public FlipperSetPos(Flipper m_flipper, Shooter m_shooter, Shoulder m_shoulder) {
    flipper = m_flipper;
    shooter = m_shooter;
    shoulder = m_shoulder;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(m_flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //System.out.print("Thisworking");
      flipper.setFlipperSetpoint(30);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
