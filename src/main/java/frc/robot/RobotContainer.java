// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightSubsytem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final TelescopingSubsystem m_telescopingSubsystem = new TelescopingSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  private final LightSubsytem m_LightSubsytem = new LightSubsytem();

  // Retained command handles

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  CommandXboxController m_armController =
      new CommandXboxController(OIConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("AutoWaitTime", AutoConstants.autoWaitTime);
    SmartDashboard.putNumber("AutoDriveSpeed", AutoConstants.autoDriveSpeed);
    SmartDashboard.putNumber("AutoDriveTime", AutoConstants.autoDriveDuration);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        Commands.run(
            () ->
                m_robotDrive.curvatureDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));
    
    m_telescopingSubsystem.setDefaultCommand(Commands.run(() -> m_telescopingSubsystem.setArmSpeed(m_armController.getRightY()), m_telescopingSubsystem));

    m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.setArmSpeed(m_armController.getLeftY()), m_armSubsystem));

    m_clawSubsystem.setDefaultCommand(Commands.run(() -> m_clawSubsystem.setClawSpeed(m_armController.getLeftTriggerAxis() - m_armController.getRightTriggerAxis()), m_clawSubsystem));

    //Initialize the lights
    m_LightSubsytem.InitializeLights();
    
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_armController.a().whileTrue(Commands.runOnce(() -> m_armSubsystem.setArmSpeed(ClawConstants.clawHoldSpeed)));

    m_driverController.a().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(0)));
    m_driverController.x().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(1)));
    m_driverController.y().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(2)));

    m_driverController.start().whileTrue(Commands.runOnce(() -> m_robotDrive.Balance(), m_robotDrive))
      .onFalse(Commands.runOnce(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive));

    // While holding Right Trigger, drive at half speed
    m_driverController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
          new ParallelRaceGroup(
            Commands.waitSeconds(SmartDashboard.getNumber("AutoWaitTime", AutoConstants.autoWaitTime)),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
          ),
          new ParallelRaceGroup(
            Commands.waitSeconds(SmartDashboard.getNumber("AutoDriveTime", AutoConstants.autoDriveDuration)),
            Commands.run(() -> m_robotDrive.arcadeDrive(SmartDashboard.getNumber("AutoDriveSpeed", AutoConstants.autoDriveSpeed), 0), m_robotDrive)
          )
        );
  }
}
