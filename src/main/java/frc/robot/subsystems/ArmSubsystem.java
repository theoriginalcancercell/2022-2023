package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ArmSubsystem extends SubsystemBase {
    
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    AnalogPotentiometer verticalPotentiometer = new AnalogPotentiometer(ArmConstants.analogVerticalPotentiometerPort);

    double targetAngle = GetArmAngle();

    public void VerticalMovement(double angle){
        /* Input angle is in degrees
         * This will calculate the current length of the actuator and utilizes that to find the angle
         * Then for now will set the motor to move to the desired position leaving a threshold of error
         * Returns true if we have reached the desired point
         */

        targetAngle = angle;
    }

    @Override
    public void periodic() {
        if(Math.abs(Math.toRadians(targetAngle) - GetArmAngle()) < ArmConstants.verticalMovementTargetThreshold){
            StopArmVertical();

            return;
        }

        //Find the current angle
        double currentAngle = GetArmAngle();
        
        //Moves towards height and will be stopped by command system once within the target threshold
        targetAngle = Math.toRadians(targetAngle);

        int direction = targetAngle - currentAngle > 0 ? 1 : -1;

        m_armVertical.set(ArmConstants.actuatorSpeed * direction);
    }

    public double GetArmAngle(){
        double potentiometerOutput = verticalPotentiometer.get();
        
        potentiometerOutput -= ArmConstants.potentiometerMinValue;
        potentiometerOutput /= (ArmConstants.potentiometerMaxValue - ArmConstants.potentiometerMinValue);

        double currentActuatorLength = 12 * potentiometerOutput + ArmConstants.actuatorClosedLength;
        
        //Law of Cosines to find the current angle
        double currentAngle = Math.acos((Math.pow(ArmConstants.actuatorMountDistanceToArmPivot, 2) + Math.pow(ArmConstants.armPivotToArmActuatorMount, 2) - Math.pow(currentActuatorLength,2))
                               /(2 * ArmConstants.actuatorMountDistanceToArmPivot * ArmConstants.armPivotToArmActuatorMount));

        return currentAngle;
    }

    public void StopArmVertical(){
        m_armVertical.stopMotor();

        targetAngle = GetArmAngle();
    }

    public void setArmSpeed(double input) {
        m_armVertical.set(input * ArmConstants.actuatorSpeed);

        targetAngle = GetArmAngle();
    }
}
