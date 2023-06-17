package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    //Initialize motors from variables in constants.java
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);

    //Function to stop the arm
    public void StopArmVertical(){
        m_armVertical.stopMotor();
    }

    //Sets the speed of the arm motor with speed scaler
    public void setArmSpeed(double input) {
        m_armVertical.set(-input * ArmConstants.actuatorSpeed);
    }

    //Sets the speed of the arm motor with no additional scaler
    public void setArmSpeedDirect(double input) {
        m_armVertical.set(input);
    }
}
