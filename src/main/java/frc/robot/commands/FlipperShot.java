// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Flipper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipperShot extends SequentialCommandGroup {
  Intake intake;
  Shooter shooter;
  Shoulder shoulder; 
  Flipper flipper;
  //double counter = 0;

  /** Creates a new FlipperShot. */
  public FlipperShot(Intake m_intake, Shooter m_shooter, Shoulder m_shoulder, Flipper m_flipper) {
    intake = m_intake;
    shooter = m_shooter;
    shoulder = m_shoulder;
    flipper = m_flipper;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*addCommands(
      if (shooter.atSetpoint() && shoulder.atShoulderSetpoint()) {
        intake.setBeltSpeed(1);
      }
      wait(1000);
      flipper.moveFlipperToPos(30);
    ); */
  }
}
