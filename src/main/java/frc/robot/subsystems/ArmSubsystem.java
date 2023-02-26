package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ArmSubsystem extends SubsystemBase {
    
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    AnalogPotentiometer verticalPotentiometer = new AnalogPotentiometer(ArmConstants.analogVerticalPotentiometerPort);

    public void StopArmVertical(){
        m_armVertical.stopMotor();
    }

    public void setArmSpeed(double input) {
        m_armVertical.set(input * ArmConstants.actuatorSpeed);
    }
}
