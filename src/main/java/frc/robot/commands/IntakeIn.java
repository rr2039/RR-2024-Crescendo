// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.LEDEffects;
import frc.utils.LEDUtility;
import frc.utils.LEDEffects.LEDEffect;

// Move flap to ground pos

public class IntakeIn extends Command {
  Shoulder shoulder;
  Intake intake;
  Shooter shooter;
  LEDUtility ledUtil;
  XboxController driver;
  XboxController operator;

  int extraRuntime = 0;

  /** Creates a new IntakeIn. */
  public IntakeIn(Intake m_intake, Shoulder m_shoulder, Shooter m_shooter, LEDUtility m_ledUtility, XboxController m_driverController, XboxController m_operatorController) {
    shoulder = m_shoulder;
    intake = m_intake;
    shooter = m_shooter;
    ledUtil = m_ledUtility;
    driver = m_driverController;
    operator = m_operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extraRuntime = 0;
    ledUtil.setAll(LEDEffect.SOLID, Color.kFirstRed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shoulder.isHome()) {
      if (!intake.hasNote()) {
        // This is negative to remove the need to invert spark
        intake.setIntakeSpeed(-1);
        intake.setBeltSpeed(0.35);
        intake.goToGround();
        if (intake.atGround() && intake.intakingNote()) {
          ledUtil.setAll(LEDEffect.FLASH, LEDEffects.rrGreen);
        }
      } else {
        intake.setBeltSpeed(0);
        intake.setIntakeSpeed(0); 
        intake.setFlapperSetpoint(IntakeConstants.flapperHome);
        if (extraRuntime % 5 == 0) {
          driver.setRumble(RumbleType.kBothRumble, 0);
          operator.setRumble(RumbleType.kBothRumble, 0);
        } else {
          driver.setRumble(RumbleType.kBothRumble, 1);
          operator.setRumble(RumbleType.kBothRumble, 1);
        }

        ledUtil.setAll(LEDEffect.SOLID, LEDEffects.rrGreen);
        extraRuntime++;
      }
    } else {
      shoulder.goHome();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driver.setRumble(RumbleType.kBothRumble, 0);
    operator.setRumble(RumbleType.kBothRumble, 0);
    if (interrupted) {
      intake.setBeltSpeed(0);
      intake.setIntakeSpeed(0); 
      intake.goHome();
    }
    //LEDEffects.setSolidColor(ledUtil.getStrip(0), Color.kBlack);
    ledUtil.setDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return intake.hasNote();
    // 1 seconds times 50 cycles per second
    return extraRuntime == (1 * 50);
  }
}
