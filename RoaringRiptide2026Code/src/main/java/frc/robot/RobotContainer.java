// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  // The operator's button board
  private final CommandGenericHID m_buttonBoard = 
      new CommandGenericHID(OIConstants.kButtonBoardPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureBindings();

    // Build an auto chooser. Commands.none() is default option
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive).withName("Robot Drive Default"));

    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_launcher);

    SmartDashboard.putNumber("Bat Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putData("Intake", m_intake.runIntakeCommand().withName("Intake - Intaking"));
    SmartDashboard.putData("Outtake", m_intake.runOuttakeCommand().withName("Intake - Outtaking"));

    SmartDashboard.putData("Feeder", m_launcher.runFeederCommand().withName("Launcher - Feeding and Launching"));
    SmartDashboard.putData("Flywheel", m_launcher.runFlywheelCommand().withName("Launcher - Spinning up Flywheel"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // xbox controller 🎮
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    
    // Start Button -> Zero swerve heading
    // needs to become button board

    // Right Trigger -> Run fuel intake in reverse
    // m_driverController
    //   .rightTrigger(OIConstants.kTriggerButtonThreshold)
    //   .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Run fuel intake in reverse
    // m_driverController
    //   .leftTrigger(OIConstants.kTriggerButtonThreshold)
    //   .whileTrue(m_intake.runOuttakeCommand());

    // button board 😀
    
    // intake + conveyor to intake fuel efficiently (old L1 button)
    m_buttonBoard.button(5).toggleOnTrue(m_intake.runIntakeCommand());
    
    // full launching mechanism (conveyor + launcher) (old L2 button)
    m_buttonBoard.button(3)
    .toggleOnTrue(m_launcher.runLauncherCommand()
    .alongWith(m_intake.runConveyorCommand()));
    
    // outtake + reverse conveyor to feed or for other purposes (old L3 button)
    m_buttonBoard.button(4).toggleOnTrue(m_intake.runOuttakeCommand());
    
    // just feeder motor, mainly for debugging (old L4 button)
    m_buttonBoard.button(6).toggleOnTrue(m_launcher.runFeederCommand());

    // Zero swerve heading (makes robot think where it is facing is the front) (old zero button)
    m_buttonBoard.button(9).onTrue(m_robotDrive.zeroHeadingCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
