// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Intake;

public class AutoShoot extends Command {
    Shooter shooter;
    Shoulder shoulder;
    Intake intake;
    int counter = 0;
    boolean shot = false;
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter m_shooter, Shoulder m_shoulder, Intake m_intake) {
    shooter = m_shooter;
    shoulder = m_shoulder;
    intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    shot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shoulder.setShoulderSetpoint(80);
    //shooter.setShooterSetpoint(1200);
    if (shooter.atSetpoint() && intake.hasNote()) {
      intake.setBeltSpeed(1);
      shot = true;
    }
    if (shot) {
      counter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setBeltSpeed(0);
    shooter.setShooterSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == (1 * 50);
  }
}
