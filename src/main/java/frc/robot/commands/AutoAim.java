// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

public class AutoAim extends Command {
  DriveSubsystem drive;
  PoseEstimatorSubsystem poseEst;

  XboxController driverController;

  PIDController turnPID = new PIDController(0, 0, 0);

  /** Creates a new AutoAim. */
  public AutoAim(DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEst, XboxController m_driveCont) {
    drive = m_drive;
    poseEst = m_poseEst;
    driverController = m_driveCont;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult tag = poseEst.getLatestTag();
    if (tag.hasTargets() && isSpeakerTag(tag.getBestTarget().getFiducialId())) {
      System.out.println(tag.getBestTarget().getYaw());
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(poseEst.getLatestTag().getBestTarget().getPitch()));
      drive.drive(MathUtil.applyDeadband(Math.pow(driverController.getRawAxis(0), 2) * Math.signum(driverController.getRawAxis(0)), OIConstants.kDriveDeadband) / (range/3),
            MathUtil.applyDeadband(Math.pow(-driverController.getRawAxis(1), 2) * Math.signum(driverController.getRawAxis(0)), OIConstants.kDriveDeadband) / (range/3),
            turnPID.calculate(tag.getBestTarget().getYaw()),
            true,
            false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean isSpeakerTag(double tag) {
    return (tag == 7 || tag == 8 || tag == 3 || tag == 4);
  }
}
