// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Center1CIR extends SequentialCommandGroup {
  /** Creates a new Center1CIR. */
  public Center1CIR(Shooter shooter, Shoulder shoulder, Intake intake, PoseEstimatorSubsystem poseEst, LEDUtility ledUtil, XboxController driver, XboxController oper, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(shooter, shoulder, intake).withTimeout(2)
    );
  }
}
