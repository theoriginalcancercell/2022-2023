package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    //Gets the claw motor on intialize
    MotorController clawMotor = new PWMVictorSPX(ClawConstants.clawMotorPort);

    //Funciton to set the speed of the claw motor with speed scaler
    public void setClawSpeed(double input){
        clawMotor.set(input * ClawConstants.clawMotorSpeed);
    }

    //Function to set the claw speed without speed scaler
    public void setClawSpeedDirect(double input){
        clawMotor.set(input);
    }

    //Function to stop the claw motor
    public void stopClaw(){
        clawMotor.stopMotor();
    }
}
