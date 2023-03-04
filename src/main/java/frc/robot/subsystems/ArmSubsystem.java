package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);

    public void StopArmVertical(){
        m_armVertical.stopMotor();
    }

    public void setArmSpeed(double input) {
        m_armVertical.set(-input * ArmConstants.actuatorSpeed);
    }

    public void setArmSpeedDirect(double input) {
        m_armVertical.set(input);
    }
}
