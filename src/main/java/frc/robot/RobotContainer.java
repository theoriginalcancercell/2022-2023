// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.LightSubsytem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  //private final LightSubsytem m_LightSubsytem = new LightSubsytem();

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
    //Adds variables for autonomous to the smartdashboard
    SmartDashboard.putNumber("AutoWaitTime", AutoConstants.autoWaitTime);
    SmartDashboard.putNumber("AutoDriveSpeed", AutoConstants.autoDriveSpeed);
    SmartDashboard.putNumber("AutoDriveTime", AutoConstants.autoDriveDuration);

    SmartDashboard.putNumber("AutoArmRaiseDuration", AutoConstants.autoArmRaiseDuration);
    SmartDashboard.putNumber("AutoArmRaiseSpeed", AutoConstants.autoArmRaiseSpeed);

    SmartDashboard.putNumber("AutoArmOutDuration", AutoConstants.autoArmOutDuration);
    SmartDashboard.putNumber("AutoArmOutSpeed", AutoConstants.autoArmOutSpeed);

    SmartDashboard.putNumber("AutoArmDownDuration", AutoConstants.autoArmDownDuration);
    SmartDashboard.putNumber("AutoArmDownSpeed", AutoConstants.autoArmDownSpeed);
    
    SmartDashboard.putNumber("BalanceTime", AutoConstants.balanceTime);


    //CameraServer.startAutomaticCapture(0);


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // Sets up curvature drive based on the left stick axis of motion
        Commands.run(
            () ->
                m_robotDrive.curvatureDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));
    
    //Makes the telescoping mechanism run on the right stick's up and down
    m_telescopingSubsystem.setDefaultCommand(Commands.run(() -> m_telescopingSubsystem.setArmSpeed(m_armController.getRightY()), m_telescopingSubsystem));

    //Sets the arm subsystems to run the set arm speed function based on the left stick's up and down
    m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.SetArmSpeed(m_armController.getLeftY()), m_armSubsystem));

    //Sets the claw speed equal to the difference of the inputs of the triggers so one pushes game pieces out and one pulls them in
    m_clawSubsystem.setDefaultCommand(Commands.run(() -> m_clawSubsystem.setClawSpeed(m_armController.getLeftTriggerAxis() - m_armController.getRightTriggerAxis()), m_clawSubsystem));

    // //Initialize the lights
    // m_LightSubsytem.InitializeLights();
    
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
    //Links the right bumper to turn the claw motors holding pieces in place
    m_armController.rightBumper().whileTrue(Commands.run(() -> m_clawSubsystem.setClawSpeed(ClawConstants.clawHoldSpeed)));

    //When the left bumper is pressed the claw throws at full force
    m_armController.leftBumper().whileTrue(Commands.run(() -> m_clawSubsystem.setClawSpeedDirect(1)));

    //Sets the led lights based on button inputs from the driver controller
    // m_driverController.a().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(0)));
    // m_driverController.x().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(1)));
    // m_driverController.y().onTrue(Commands.runOnce(() -> m_LightSubsytem.ChangeLightState(2)));

    //Runs the autobalance function while holding start then zeroes the motors on release
    m_driverController.start().whileTrue(Commands.print("running").andThen(new RepeatCommand(Commands.run(() -> m_robotDrive.Balance(), m_robotDrive))))
      .onFalse(Commands.runOnce(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive));

    //Rotate towards the starting direction or the other direction when bumpers are held
    m_driverController.rightBumper()
      .whileTrue(Commands.run(() -> m_robotDrive.RotateTo(0)));
    m_driverController.leftBumper()
      .whileTrue(Commands.run(() -> m_robotDrive.RotateTo(180)));

    // While holding Right Trigger, drive at half speed
    m_driverController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));
        
    // While holding left drigger, drive at 1/4 speed
    m_driverController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.25)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));
  }

  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(
    //       new ParallelRaceGroup(
            // Commands.waitSeconds(SmartDashboard.getNumber("AutoWaitTime", AutoConstants.autoWaitTime)),
            // Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
    //       ),
    //       new ParallelRaceGroup(
    //         Commands.waitSeconds(SmartDashboard.getNumber("AutoDriveTime", AutoConstants.autoDriveDuration)),
    //         Commands.run(() -> m_robotDrive.arcadeDrive(SmartDashboard.getNumber("AutoDriveSpeed", AutoConstants.autoDriveSpeed), 0), m_robotDrive)
    //       )
    //     );
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        Commands.waitSeconds(SmartDashboard.getNumber("AutoWaitTime", AutoConstants.autoWaitTime)),
        Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
      Commands.waitSeconds(SmartDashboard.getNumber("AutoArmRaiseDuration", AutoConstants.autoArmRaiseDuration)),
            Commands.run(() -> m_armSubsystem.setArmSpeedDirect(SmartDashboard.getNumber("AutoArmRaiseSpeed", AutoConstants.autoArmRaiseSpeed)))
              .finallyDo((end) -> m_armSubsystem.setArmSpeedDirect(0)),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
        Commands.waitSeconds(SmartDashboard.getNumber("AutoArmOutDuration", AutoConstants.autoArmOutDuration)),
            Commands.run(() -> m_telescopingSubsystem.setArmSpeedDirect(SmartDashboard.getNumber("AutoArmOutSpeed", AutoConstants.autoArmOutSpeed)), m_telescopingSubsystem)
              .finallyDo((end) -> m_telescopingSubsystem.setArmSpeedDirect(0)),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
        Commands.waitSeconds(1),
            Commands.run(() -> m_clawSubsystem.setClawSpeed(.75), m_clawSubsystem)
              .finallyDo((end) -> m_clawSubsystem.setClawSpeed(0)),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
        Commands.waitSeconds(SmartDashboard.getNumber("AutoArmDownDuration", AutoConstants.autoArmDownDuration)),
            Commands.run(() -> m_armSubsystem.setArmSpeedDirect(SmartDashboard.getNumber("AutoArmDownSpeed", AutoConstants.autoArmDownSpeed)), m_armSubsystem),
            Commands.run(() -> m_telescopingSubsystem.setArmSpeedDirect(SmartDashboard.getNumber("AutoArmInSpeed", AutoConstants.autoArmInSpeed)), m_telescopingSubsystem),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
        Commands.waitSeconds(.1),
            Commands.run(() -> m_armSubsystem.setArmSpeedDirect(0), m_armSubsystem),
            Commands.run(() -> m_telescopingSubsystem.setArmSpeedDirect(0), m_telescopingSubsystem),
            Commands.run(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
      ),
      new ParallelRaceGroup(
            Commands.waitSeconds(SmartDashboard.getNumber("AutoDriveTime", AutoConstants.autoDriveDuration)),
            Commands.run(() -> m_robotDrive.arcadeDrive(-SmartDashboard.getNumber("AutoDriveSpeed", AutoConstants.autoDriveSpeed), 0), m_robotDrive)
          ),
      new ParallelRaceGroup(
        Commands.waitSeconds(SmartDashboard.getNumber("BalanceTime", AutoConstants.balanceTime)),
        Commands.run(() -> m_robotDrive.Balance(), m_robotDrive)
      )
    );
  }
}
