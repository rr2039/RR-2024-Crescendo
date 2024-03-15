// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Mobility extends SequentialCommandGroup {
  /** Creates a new Mobility. */
  public Mobility(PoseEstimatorSubsystem poseEst) {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Mobility");
    Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(path.getPreviewStartingHolonomicPose()));

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        path.flipPath();
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      resetPose,
      AutoBuilder.followPath(path));
  }
}
