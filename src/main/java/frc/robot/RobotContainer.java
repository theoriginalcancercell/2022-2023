// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
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
    
    m_telescopingSubsystem.setDefaultCommand(Commands.run(() -> m_telescopingSubsystem.setArmSpeed(m_armController.getLeftY()), m_telescopingSubsystem));

    m_clawSubsystem.setDefaultCommand(Commands.run(() -> m_clawSubsystem.setClawSpeed(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()), m_clawSubsystem));

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
    // Binds the four buttons to arm positions
    m_armController.b().onTrue(Commands.runOnce(() -> m_armSubsystem.VerticalMovement(ArmConstants.closedAngle)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));
    m_armController.a().onTrue(Commands.runOnce(() -> m_armSubsystem.VerticalMovement(ArmConstants.maxAngle)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));
    m_armController.y().onTrue(Commands.runOnce(() -> m_armSubsystem.VerticalMovement(ArmConstants.levelOneAngle)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));
    m_armController.x().onTrue(Commands.runOnce(() -> m_armSubsystem.VerticalMovement(ArmConstants.levelTwoAngle)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));

    m_armController.rightBumper().whileTrue(Commands.runOnce(() -> m_armSubsystem.setArmSpeed(-1)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));
    m_armController.leftBumper().whileTrue(Commands.runOnce(() -> m_armSubsystem.setArmSpeed(1)))
        .onFalse(Commands.runOnce(() -> m_armSubsystem.StopArmVertical()));

    m_driverController.a().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(0)));
    m_driverController.x().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(1)));
    m_driverController.y().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(2)));

    // While holding R1, drive at half speed
    m_driverController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
System.out.println("getting command");

    return //Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive).raceWith(Commands.waitSeconds(SmartDashboard.getNumber("AutoWaitTime", AutoConstants.autoWaitTime)))
        //.andThen(
          Commands.run(() -> m_robotDrive.arcadeDrive(.25, 0), m_robotDrive);
        //.raceWith(Commands.waitSeconds(SmartDashboard.getNumber("AutoDriveTime", AutoConstants.autoDriveDuration)));

        //Commands.run(() -> System.out.println("Hi"));
  }
}
