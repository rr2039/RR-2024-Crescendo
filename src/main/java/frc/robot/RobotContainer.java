// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.Amp1CIR;
import frc.robot.autos.Amp2CIR;
import frc.robot.autos.Amp4CIR;
import frc.robot.autos.Center1CIR;
import frc.robot.autos.Center2CIR;
import frc.robot.autos.Center4CIR;
import frc.robot.autos.Center5CIR;
import frc.robot.autos.Mobility;
import frc.robot.autos.SPSourceSide;
import frc.robot.autos.Source1CIR;
import frc.robot.autos.Source2CIR;
import frc.robot.autos.Source3CIR;
import frc.robot.autos.Source4CIR;
import frc.robot.autos.Test;
import frc.robot.commands.AmpShot;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimNote;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShooterFeed;
import frc.robot.commands.ShooterOn;
import frc.robot.commands.ShoulderSetPos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.LEDEffects;
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
    public LEDUtility m_ledUtil = new LEDUtility(0);

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake(m_ledUtil);
  public PoseEstimatorSubsystem m_poseEst = new PoseEstimatorSubsystem(m_robotDrive);
  private final Shoulder m_shoulder = new Shoulder(m_intake::hasNote, m_poseEst);
  private final Shooter m_shooter = new Shooter(m_intake::hasNote, m_poseEst, m_ledUtil);
  private final Climber m_climber = new Climber();
  //private final Flipper m_flipper = new Flipper(m_shoulder);

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
    m_ledUtil.addStrip(new LEDStrip(0, 15, false)); // Shoulder Left
    m_ledUtil.addStrip(new LEDStrip(16, 51, false)); // Shooter Left 1
    m_ledUtil.addStrip(new LEDStrip(52, 87, true)); // Shooter Left 2
    m_ledUtil.addStrip(new LEDStrip(88, 123, false)); // Shooter Right 1
    m_ledUtil.addStrip(new LEDStrip(124, 159, true)); // Shooter Right 2
    m_ledUtil.addStrip(new LEDStrip(160, 175, false)); // Shoulder Right
    m_ledUtil.addStrip(new LEDStrip(176, 217, false)); // Under Glow
    m_ledUtil.setDefault();

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

    NamedCommands.registerCommand("ShooterFeed", new ShooterFeed(m_intake, m_shooter, m_shoulder));
    NamedCommands.registerCommand("IntakeOn", new IntakeIn(m_intake, m_shoulder, m_shooter, m_ledUtil, m_driverController, m_operatorController));
    NamedCommands.registerCommand("AutoAim", new AutoAim(m_robotDrive, m_poseEst, m_driverController));

    // Build an auto chooser. This will use Commands.none() as the default option.
    auto_chooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    auto_chooser.addOption("Mobility Manual", new Mobility(m_poseEst));
    auto_chooser.addOption("SP Source Side Manual", new SPSourceSide(m_shooter, m_shoulder, m_intake, m_poseEst));
    auto_chooser.addOption("Test Manual", new Test(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Center4CIR Manual", new Center4CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Source3CIR Manual", new Source3CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Amp4CIR Manual", new Amp4CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Source4CIR Manual", new Source4CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Center5CIR Manual", new Center5CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Center2CIR Manual", new Center2CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Amp2CIR Manual", new Amp2CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Source2CIR Manual", new Source2CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Amp1CIR Manual", new Amp1CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Center1CIR Manual", new Center1CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));
    auto_chooser.addOption("Source1CIR Manual", new Source1CIR(m_shooter, m_shoulder, m_intake, m_poseEst, m_ledUtil, m_driverController, m_operatorController, m_robotDrive));


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
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new AutoAimNote(m_robotDrive, m_driverController));

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
    new POVButton(m_operatorController, 90)
        .whileTrue(new AmpShot(m_shooter, m_shoulder, m_driverController));
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
