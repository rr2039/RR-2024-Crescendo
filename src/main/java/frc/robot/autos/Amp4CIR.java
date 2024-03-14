// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj.DriverStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp4CIR extends SequentialCommandGroup {
  /** Creates a new Source3CIR. */
  public Amp4CIR(Shooter shooter, Shoulder shoulder, Intake intake, PoseEstimatorSubsystem poseEst, LEDUtility ledUtil, XboxController driver, XboxController oper, DriveSubsystem drive) {
    PathPlannerPath note1Path = PathPlannerPath.fromPathFile("Amp #1 CIR");
    PathPlannerPath note4Path = PathPlannerPath.fromPathFile("Intake #4 CIR");
    PathPlannerPath shoot4Path = PathPlannerPath.fromPathFile("Shoot #4 CIR");
    PathPlannerPath note5Path = PathPlannerPath.fromPathFile("Intake #5 CIR");
    PathPlannerPath shoot5Path = PathPlannerPath.fromPathFile("Shoot #5 CIR");
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        note1Path.flipPath(); 
        note4Path.flipPath();
        shoot4Path.flipPath();
        note5Path.flipPath();
        shoot5Path.flipPath();
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(2)),
        AutoBuilder.followPath(note1Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note4Path)),
        AutoBuilder.followPath(shoot4Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake),
        new ParallelCommandGroup(new IntakeIn(intake, shoulder, shooter, ledUtil, driver, oper).withTimeout(3), AutoBuilder.followPath(note5Path)),
        AutoBuilder.followPath(shoot5Path),
        new AutoAim(drive, poseEst, driver),
        new WaitCommand(1),
        new AutoShoot(shooter, shoulder, intake)
        );
      }
    }
