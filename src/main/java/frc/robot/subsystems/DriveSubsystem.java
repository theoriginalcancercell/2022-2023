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

  //Initialize all the motors into motor groups
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

  //Gets the gyro
  ADIS16448_IMU gyro = new ADIS16448_IMU();

  //Creates the robot's drive system
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    //Zeros out the motor output
    m_drive.arcadeDrive(0, 0);
    
    //This calibrates and intializes the gyro
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

  //Same as arcade drive but this rotates in place when forward input is less than 0.05
  public void curvatureDrive(double fwd, double rot) {
    m_drive.curvatureDrive(-scaleJoysticks(fwd), rot, Math.abs(fwd) < 0.05);
  }

  //This scales joystick input so they feel more natural by cubing them
  public double scaleJoysticks(double input){
    return Math.pow(input, 3);
  }

  //Should auto balance the robot on the charge station but never quite got working rights
  public void Balance(){
    //Checks if we are within a certain threshold of level then zeros the motors and skips the function if true
    if (Math.abs(gyro.getGyroAngleY()) < DriveConstants.balancingThreshold) {
      arcadeDrive(0, 0);

      return;
    }
    
    //Gets a temporary motor speed value
    //Initially set so that it will become a fraction after a certain threshold
    double motorSpeed = gyro.getGyroAngleY() / DriveConstants.balancingFractioningThreshold + Math.copySign(0.3, gyro.getGyroAngleY());

    //If the value of motorspeed is greater than 1 we set it to one with its original sign
    if (Math.abs(motorSpeed) > 1) {
      motorSpeed = Math.copySign(1, motorSpeed);
    }

    //Drives the robot in the desired direction to level
    arcadeDrive(motorSpeed, 0);
  }
  
  //This will rotate the robot towards the desired angle for the given tick
  public void RotateTo(double angle){
    //Leaves the function if we fall within a certain threshold
    if (Math.abs(gyro.getGyroAngleZ() + angle) < DriveConstants.rotatingThreshold) {
      arcadeDrive(0, 0);

      return;
    }

    //Gets a temporary motor speed value
    //Initially set so that it will become a fraction after a certain threshold
    //A small value is added that has the same sign as the input this is the minmum to get the motors to move
    double motorSpeed = (gyro.getGyroAngleZ() + angle) / DriveConstants.rotatingFractioningThreshold + Math.copySign(0.2, (gyro.getGyroAngleZ() + angle));

    //If the value of motorspeed is greater than 1 we set it to one with its original sign
    if (Math.abs(motorSpeed) > 1) {
      motorSpeed = Math.copySign(1, motorSpeed);
    }
    
    //Sets the direction with curvature drive so we can rotate in place
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
