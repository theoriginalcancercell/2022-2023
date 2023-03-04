// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new PWMSparkMax(DriveConstants.kLeftMotor1Port),
          new PWMSparkMax(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new PWMSparkMax(DriveConstants.kRightMotor1Port),
          new PWMSparkMax(DriveConstants.kRightMotor2Port));

  ADIS16448_IMU gyro = new ADIS16448_IMU();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    m_drive.arcadeDrive(0, 0);
    
    gyro.calibrate();
}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(-fwd, rot);
  }

  public void curvatureDrive(double fwd, double rot) {
    m_drive.curvatureDrive(-scaleJoysticks(fwd), rot, Math.abs(fwd) < 0.05);
  }

  public double scaleJoysticks(double input){
    return Math.pow(input, 3);
  }

  public void Balance(){
    if (Math.abs(gyro.getGyroAngleY()) < DriveConstants.balancingThreshold) {
      arcadeDrive(0, 0);

      return;
    }
    
    double motorSpeed = gyro.getGyroAngleY() / DriveConstants.balancingFractioningThreshold + Math.copySign(0.35, gyro.getGyroAngleY());

    if (Math.abs(motorSpeed) > 1) {
      motorSpeed = Math.copySign(1, motorSpeed);
    }
    
    arcadeDrive(motorSpeed, 0);
  }
  
  public void RotateTo(double angle){
    
    if (Math.abs(gyro.getGyroAngleZ() + angle) < DriveConstants.rotatingThreshold) {
      arcadeDrive(0, 0);

      return;
    }

    double motorSpeed = (gyro.getGyroAngleZ() + angle) / DriveConstants.rotatingFractioningThreshold + Math.copySign(0.2, (gyro.getGyroAngleZ() + angle));

    if (Math.abs(motorSpeed) > 1) {
      motorSpeed = Math.copySign(1, motorSpeed);
    }
    
    curvatureDrive(0, motorSpeed);
  }


  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
