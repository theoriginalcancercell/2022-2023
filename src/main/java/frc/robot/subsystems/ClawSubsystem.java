package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    MotorController clawMotor = new PWMVictorSPX(ClawConstants.clawMotorPort);

    public void setClawSpeed(double input){
        clawMotor.set(input * ClawConstants.clawMotorSpeed);
    }

    public void stopClaw(){
        clawMotor.stopMotor();
    }
}
