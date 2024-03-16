// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.RestoreAction;

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
import frc.robot.commands.AutonomousAim;
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
public class Center5CIR extends SequentialCommandGroup {
  /** Creates a new Center5CIR. */
  public Center5CIR(Shooter shooter, Shoulder shoulder, Intake intake, PoseEstimatorSubsystem poseEst, LEDUtility ledUtil, XboxController driver, XboxController oper, DriveSubsystem drive) {
    PathPlannerPath note1Path = PathPlannerPath.fromPathFile("Intake #1 CIR");
    PathPlannerPath note2Path = PathPlannerPath.fromPathFile("Intake #2 CIR");
    PathPlannerPath note3Path = PathPlannerPath.fromPathFile("Intake #3 CIR");
    PathPlannerPath note8Path = PathPlannerPath.fromPathFile("Center #8 CIR");
    PathPlannerPath shoot8Path = PathPlannerPath.fromPathFile("Shoot #8 CIR");
    Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(note1Path.getPreviewStartingHolonomicPose()));


    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        note1Path.flipPath(); 
        note2Path.flipPath();
        note3Path.flipPath();
        note8Path.flipPath();
        shoot8Path.flipPath();
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        resetPose,
        new AutoShoot(shooter, shoulder, intake), 
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note1Path)), 
        new AutonomousAim(drive, poseEst, driver).withTimeout(1), 
        new WaitCommand(1), 
        new AutoShoot(shooter, shoulder, intake), 
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note2Path)),
        new AutonomousAim(drive, poseEst, driver).withTimeout(1), 
        new WaitCommand(1), 
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note3Path)),
        new AutonomousAim(drive, poseEst, driver).withTimeout(1), 
        new WaitCommand(1), 
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note8Path)), 
        AutoBuilder.followPath(shoot8Path),
        new AutonomousAim(drive, poseEst, driver).withTimeout(1), 
        new WaitCommand(1), 
        new AutoShoot(shooter, shoulder, intake)
        );
  }
}
