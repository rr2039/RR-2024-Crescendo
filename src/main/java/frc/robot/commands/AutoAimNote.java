// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

public class AutoAimNote extends Command {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable limelightTable = inst.getTable("limelight");
  private NetworkTableEntry tv, tx, tclass;

  private DriveSubsystem drive;
  private XboxController dController;

  PIDController turnPID = new PIDController(0.0125, 0, 0.0001); //2.5p
  /** Creates a new AutoAimNote. */
  public AutoAimNote(DriveSubsystem m_drive, XboxController m_driveCont) {
    drive = m_drive;
    dController = m_driveCont;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tx = limelightTable.getEntry("tx");
    tv = limelightTable.getEntry("tv");
    tclass = limelightTable.getEntry("tclass");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tv.getNumber(0).doubleValue() == 1.0 && tclass.getString("").equals("note")) {
      double x_pos = tx.getNumber(0).doubleValue();
       drive.powDrive(MathUtil.applyDeadband(dController.getRawAxis(0), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(-dController.getRawAxis(1), OIConstants.kDriveDeadband),
            turnPID.calculate(x_pos, 0),
            true,
            false,
            false);
    } else {
      drive.powDrive(
                MathUtil.applyDeadband(dController.getRawAxis(0), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-dController.getRawAxis(1), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-dController.getRawAxis(4), OIConstants.kDriveDeadband),
                true, false, true);
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
}
