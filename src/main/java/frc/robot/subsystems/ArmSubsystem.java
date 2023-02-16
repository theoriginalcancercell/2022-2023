package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ArmSubsystem extends SubsystemBase {
    
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    AnalogPotentiometer verticalPotentiometer = new AnalogPotentiometer(ArmConstants.analogVerticalPotentiometerPort);

    public MotorController m_armTelescoping = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    
    public CommandBase VerticalGoTo(double angle) {
        //Moves arm until the arm is within the correct threshold
        return this.run(() -> VerticalMovement((angle))).until(() -> Math.abs(angle - GetArmAngle()) < ArmConstants.verticalMovementTargetThreshold );
    }

    public void VerticalMovement(double angle){
        /* Input angle is in degrees
         * This will calculate the current length of the actuator and utilizes that to find the angle
         * Then for now will set the motor to move to the desired position leaving a threshold of error
         * Returns true if we have reached the desired point
         */

        //Find the current length
        double currentAngle = GetArmAngle();

        //Moves towards height and will be stopped by command system once within the target threshold
        angle = Math.toRadians(angle);

        int direction = angle - currentAngle > 0 ? 1 : -1;

        m_armVertical.set(ArmConstants.actuatorSpeed * direction);
    }

    public double GetArmAngle(){
        double potentiometerOutput = verticalPotentiometer.get();

        potentiometerOutput -= ArmConstants.potentiometerMinValue;
        potentiometerOutput /= (ArmConstants.potentiometerMaxValue - ArmConstants.potentiometerMinValue);

        double currentActuatorLength = 12 * potentiometerOutput;

        //Law of Cosines to find the current angle
        double currentAngle = Math.acos((Math.pow(ArmConstants.actuatorMountDistanceToArmPivot, 2) + Math.pow(ArmConstants.armPivotToArmActuatorMount, 2) - Math.pow(currentActuatorLength,2))
                               /(2 * ArmConstants.actuatorMountDistanceToArmPivot * ArmConstants.armPivotToArmActuatorMount));

        return currentAngle;
    }

    public CommandBase TelescopeArm(double input){
        return this.run(() -> m_armTelescoping.set(input * ArmConstants.spoolSpeed));
    }
}
