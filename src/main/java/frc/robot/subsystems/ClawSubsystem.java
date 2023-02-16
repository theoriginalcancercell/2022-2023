package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    MotorController clawMotor = new PWMVictorSPX(ClawConstants.clawMotorPort);

    public CommandBase RunClaw(double input){
        return this.run(() -> clawMotor.set(input * ClawConstants.clawMotorSpeed));
    }
}
