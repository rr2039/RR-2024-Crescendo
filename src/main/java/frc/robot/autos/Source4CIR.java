// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.IntakeIn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Source4CIR extends SequentialCommandGroup {
  /** Creates a new Source4CIR. */
  public Source4CIR(Shooter shooter, Shoulder shoulder, Intake intake, PoseEstimatorSubsystem poseEst, LEDUtility ledUtil, XboxController driver, XboxController oper, DriveSubsystem drive) {
    PathPlannerPath note8Path = PathPlannerPath.fromPathFile("Intake #8 CIR");
    PathPlannerPath shoot8Path = PathPlannerPath.fromPathFile("Shoot #8 CIR");
    PathPlannerPath note7Path = PathPlannerPath.fromPathFile("Intake #7 CIR");
    PathPlannerPath shoot7Path = PathPlannerPath.fromPathFile("Shoot #7 CIR");
    PathPlannerPath note6Path = PathPlannerPath.fromPathFile("Intake #6 CIR");
    PathPlannerPath shoot6Path = PathPlannerPath.fromPathFile("Shoot #6 CIR");
    Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(note8Path.getPreviewStartingHolonomicPose()));



    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        note8Path.flipPath(); 
        shoot8Path.flipPath();
        note7Path.flipPath();
        shoot7Path.flipPath();
        note6Path.flipPath();
        shoot6Path.flipPath();
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        resetPose,
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(4), AutoBuilder.followPath(note8Path)),
        AutoBuilder.followPath(shoot8Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note7Path)),
        AutoBuilder.followPath(shoot7Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(4), AutoBuilder.followPath(note6Path)),
        AutoBuilder.followPath(shoot6Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake)
        );
  }
}
