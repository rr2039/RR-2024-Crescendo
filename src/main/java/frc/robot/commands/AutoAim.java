// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.utils.PoseUtils;

import static edu.wpi.first.math.util.Units.radiansToDegrees;

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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.LEDEffects;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

public class AutoAim extends Command {
  DriveSubsystem drive;
  PoseEstimatorSubsystem poseEst;

  XboxController driverController;

  PIDController turnPID = new PIDController(2.5, 0, 0.01); //1.6p

  AprilTagFieldLayout layout;

  Shoulder shoulder;
  Shooter shooter;
  LEDUtility ledUtil;

  /** Creates a new AutoAim. */
  public AutoAim(DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEst, XboxController m_driveCont, Shoulder m_shoulder, Shooter m_shooter, LEDUtility m_led) {
    drive = m_drive;
    poseEst = m_poseEst;
    driverController = m_driveCont;
    shoulder = m_shoulder;
    shooter = m_shooter;
    ledUtil = m_led;

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
      drive.powDrive(MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(-driverController.getRawAxis(1), OIConstants.kDriveDeadband),
            turnPID.calculate(-yaw.getRadians(), 0),
            true,
            false,
            false);
      if (shooter.atSetpoint() && shoulder.atShoulderSetpoint() && (-2 < radiansToDegrees(yaw.getRadians()) && radiansToDegrees(yaw.getRadians()) < 2)) {
        ledUtil.getStrip(2).setEffect(LEDEffect.SOLID);
        ledUtil.getStrip(2).setColor(LEDEffects.rrGreen);
        ledUtil.getStrip(4).setEffect(LEDEffect.SOLID);
        ledUtil.getStrip(4).setColor(LEDEffects.rrGreen);
        driverController.setRumble(RumbleType.kBothRumble, 1);
      } else {
        ledUtil.getStrip(2).setEffect(LEDEffect.SOLID);
        ledUtil.getStrip(2).setColor(Color.kBlack);
        ledUtil.getStrip(4).setEffect(LEDEffect.SOLID);
        ledUtil.getStrip(4).setColor(Color.kBlack);
        driverController.setRumble(RumbleType.kBothRumble, 0);
      }
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledUtil.getStrip(2).setEffect(LEDEffect.SOLID);
    ledUtil.getStrip(2).setColor(Color.kBlack);
    ledUtil.getStrip(4).setEffect(LEDEffect.SOLID);
    ledUtil.getStrip(4).setColor(Color.kBlack);
    driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
