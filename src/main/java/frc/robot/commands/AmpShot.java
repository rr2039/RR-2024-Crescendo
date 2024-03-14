// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class AmpShot extends Command {
  Shooter shooter;
  Shoulder shoulder;
  XboxController driver;
  /** Creates a new ShooterOn. */
  public AmpShot(Shooter m_shooter, Shoulder m_shoulder, XboxController m_driver) {
    shooter = m_shooter;
    shoulder = m_shoulder;
    driver = m_driver;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setManualOverride(true);
    shoulder.setManualOverride(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSetpoint(380);
    if (shooter.atSetpoint()) {
      driver.setRumble(RumbleType.kBothRumble, 1);
    }
    //shooter.setShooter(0.35);
    shoulder.setShoulderSetpoint(65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.setShooterSetpoint(500);
    shooter.setShooterSetpoint(0);
    driver.setRumble(RumbleType.kBothRumble, 0);
    shooter.setManualOverride(false);
    shoulder.setManualOverride(false);
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
