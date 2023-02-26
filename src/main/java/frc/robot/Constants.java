// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

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
      
    public static final double balancingThreshold = .5f;
    public static final double balancingFractioningThreshold = 10f;
  }

  public static final class ArmConstants {
    public static final int armActuatingMotorPort = 4;
    public static final int armTelescopingMotorPort = 5;

    public static final int analogVerticalPotentiometerPort = 0;

    public static final float potentiometerMaxValue = 0.8888f;
    public static final float potentiometerMinValue = 0.0276f;

    //This is the bottom actuator mount along the rear of the robot to the mounting point of the arm's pivot
    public static final float actuatorMountDistanceToArmPivot = 30.5f;
    //This is the arm pivot to the point where the arm and the actuator are connected
    public static final float armPivotToArmActuatorMount = 9.75f;
    public static final float actuatorClosedLength = 21f;

    public static final float verticalMovementTargetThreshold = .25f;

    public static final float actuatorSpeed = 0.75f;
    
    public static final float inwardTelescoping = 0.25f;
    public static final float outwardTelescoping = 0.8f;

    //Targeted angles for the vertical movement
    public static final float closedAngle = 3;
    public static final float levelOneAngle = 6;
    public static final float levelTwoAngle = 9;
    public static final float maxAngle = 11.5f;
  }

  public static final class ClawConstants{
    public static final int clawMotorPort = 6;

    public static final double clawMotorSpeed = .5;

    public static final double clawHoldSpeed = -0.1;
  }

  public static final class LEDLightsConstants{
    public static final int ledPort = 7;
    public static final int ledLength = 32;
    //0 is off 1 is cube 2 is cone
    public static final Color[] colors = new Color[] {new Color(0, 0, 0), new Color(0, 0, 255), new Color(255, 255, 0)};
  }

  public static final class AutoConstants {
    public static final double autoWaitTime = 2;
    public static final double autoDriveSpeed = 0.5;
    public static final double autoDriveDuration = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
  }
}
