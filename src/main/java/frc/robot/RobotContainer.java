// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.Mobility;
import frc.robot.autos.SPSourceSide;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShooterFeed;
import frc.robot.commands.ShooterOn;
import frc.robot.commands.ShoulderSetPos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.LEDStrip;
import frc.utils.LEDUtility;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  public PoseEstimatorSubsystem m_poseEst = new PoseEstimatorSubsystem(m_robotDrive);
  private final Shoulder m_shoulder = new Shoulder(m_intake::hasNote, m_poseEst);
  private final Shooter m_shooter = new Shooter(m_intake::hasNote, m_poseEst);
  private final Climber m_climber = new Climber();

  public LEDUtility m_ledUtil = new LEDUtility(0);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  // Auto Chooser for Dashboard
  SendableChooser<Command> auto_chooser = new SendableChooser<>();

  LinearFilter filterX = LinearFilter.singlePoleIIR(0.04, 0.02);
  LinearFilter filterY = LinearFilter.singlePoleIIR(0.04, 0.02);
  LinearFilter filterR = LinearFilter.singlePoleIIR(0.04, 0.02);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_ledUtil.addStrip(new LEDStrip(1, 8, false)); 

    // Configure the button bindings
    configureButtonBindings();
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.powDrive(
                MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true, false, true),
            m_robotDrive));

    // Build an auto chooser. This will use Commands.none() as the default option.
    auto_chooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    auto_chooser.addOption("Mobility", new Mobility());
    auto_chooser.addOption("SP Source Side", new SPSourceSide(m_shooter, m_shoulder, m_intake, m_poseEst));

    SmartDashboard.putData("Auto Chooser", auto_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER CONTROLLER
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new ShooterFeed(m_intake, m_shooter, m_shoulder));
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new AutoAim(m_robotDrive, m_poseEst, m_driverController));

    // OPERATOR CONTROLLER
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .whileTrue(new IntakeIn(m_intake, m_shoulder, m_shooter, m_ledUtil, m_driverController, m_operatorController));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .whileTrue(new IntakeOut(m_intake, m_shoulder));
    new JoystickButton(m_operatorController, Button.kA.value)
        .whileTrue(new Climb(m_climber, m_shoulder, true));
    new JoystickButton(m_operatorController, Button.kY.value)
        .whileTrue(new Climb(m_climber, m_shoulder, false));
    new POVButton(m_operatorController, 270)
        .whileTrue(new ShooterOn(m_shooter, m_shoulder, m_driverController));
    new POVButton(m_operatorController, 0)
        .onTrue(new ShoulderSetPos(m_shoulder, true));
    new POVButton(m_operatorController, 180)
        .onTrue(new ShoulderSetPos(m_shoulder, false));
    new JoystickButton(m_operatorController, Button.kX.value)
        .onTrue(new RunCommand(() -> m_shoulder.setShoulderSetpoint(35), m_shoulder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}
