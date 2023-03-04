package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class TelescopingSubsystem extends SubsystemBase {
    MotorController m_armTelescoping = new PWMSparkMax(ArmConstants.armTelescopingMotorPort);
    
    public void setArmSpeed(double input) {
        m_armTelescoping.set(-1 * (input > 0 ? ArmConstants.outwardTelescoping * -input : ArmConstants.inwardTelescoping * -input));
    }

    public void setArmSpeedDirect(double input) {
        m_armTelescoping.set(input);
    }
}
