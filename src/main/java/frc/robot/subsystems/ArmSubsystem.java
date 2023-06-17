package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ArmSubsystem extends SubsystemBase {
    
    //Initialize motors from variables in constants.java
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    AnalogPotentiometer verticalPotentiometer = new AnalogPotentiometer(ArmConstants.analogVerticalPotentiometerPort);

    //Function to stop the arm
    public void StopArmVertical(){
        m_armVertical.stopMotor();
    }

    //Sets the speed of the arm motor
    public void SetArmSpeed(double input) {
        m_armVertical.set(input * ArmConstants.actuatorSpeed);
    }
}
