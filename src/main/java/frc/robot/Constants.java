// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
  }

  public static final class ArmConstants {
    public static final int armActuatingMotorPort = 0;

    public static final int analogVerticalPotentiometerPort = 0;

    public static final float potentiometerMaxValue = 0;
    public static final float potentiometerMinValue = 0;

    //This is the bottom actuator mount along the rear of the robot to the mounting point of the arm's pivot
    public static final float actuatorMountDistanceToArmPivot = 0;
    //This is the arm pivot to the point where the arm and the actuator are connected
    public static final float armPivotToArmActuatorMount = 0;

    public static final float verticalMovementTargetThreshold = 0;

    public static final float actuatorSpeed = 0.5f;
    public static final float spoolSpeed = 0.5f;

    //Targeted angles for the vertical movement
    public static final float closedAngle = 0;
    public static final float levelOneAngle = 0;
    public static final float levelTwoAngle = 0;
    public static final float maxAngle = 0;
  }

  public static final class ClawConstants{
    public static final int clawMotorPort = 0;

    public static final double clawMotorSpeed = 1;
  }

  public static final class HatchConstants {
    public static final int kHatchSolenoidModule = 0;
    public static final int[] kHatchSolenoidPorts = new int[] {0, 1};
  }

  public static final class AutoConstants {
    public static final double kAutoDriveDistanceInches = 60;
    public static final double kAutoBackupDistanceInches = 20;
    public static final double kAutoDriveSpeed = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }
}
