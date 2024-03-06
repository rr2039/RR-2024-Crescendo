// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.utils.PoseUtils;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  PIDController turnPID = new PIDController(0.4, 0, 0.01);

  AprilTagFieldLayout layout;

  /** Creates a new AutoAim. */
  public AutoAim(DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEst, XboxController m_driveCont) {
    drive = m_drive;
    poseEst = m_poseEst;
    driverController = m_driveCont;

    turnPID.setTolerance(1);

    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //PhotonPipelineResult tag = poseEst.getLatestTag();
    //if (tag.hasTargets() && isSpeakerTag(tag.getBestTarget().getFiducialId())) {
      Rotation2d yaw = PhotonUtils.getYawToPose(poseEst.getCurrentPose(), layout.getTagPose(PoseUtils.getSpeakerTag()).get().toPose2d());
      System.out.println(yaw.getDegrees() + ": YawToPose");
      drive.drive(MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(-driverController.getRawAxis(1), OIConstants.kDriveDeadband),
            turnPID.calculate(-yaw.getRadians(), 0),
            true,
            false);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
