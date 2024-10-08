// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;

public class IntakeOut extends Command {
  Shoulder shoulder;
  Intake intake;

  /** Creates a new IntakeOut. */
  public IntakeOut(Intake m_intake, Shoulder m_shoulder) {
    shoulder = m_shoulder;
    intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shoulder.isHome() && intake.atGround()) {
      // Positive so we dont have to invert the spark
      intake.setIntakeSpeed(0.75);
      intake.setBeltSpeed(-0.25);
    } else {
      shoulder.goHome();
      intake.goToGround();;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setBeltSpeed(0);
    intake.setIntakeSpeed(0); 
    intake.goHome();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
