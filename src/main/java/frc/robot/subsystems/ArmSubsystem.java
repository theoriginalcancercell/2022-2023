package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ArmSubsystem extends SubsystemBase {
    
    MotorController m_armVertical = new PWMSparkMax(ArmConstants.armActuatingMotorPort);
    AnalogPotentiometer verticalPotentiometer = new AnalogPotentiometer(ArmConstants.analogVerticalPotentiometerPort);

    double targetLength = GetArmLength();

    boolean autoRun = false;

    public void VerticalMovement(double length){
        /* Input angle is in degrees
         * This will calculate the current length of the actuator and utilizes that to find the angle
         * Then for now will set the motor to move to the desired position leaving a threshold of error
         * Returns true if we have reached the desired point
         */

        targetLength = length;
        
        autoRun = true;
    }

    @Override
    public void periodic() {
        if (!autoRun) {
            return;
        }
        
        if(Math.abs(targetLength - GetArmLength()) < ArmConstants.verticalMovementTargetThreshold){
            StopArmVertical();

            return;
        }

        //Find the current angle
        double currentLength = GetArmLength();

        int direction = targetLength - currentLength > 0 ? 1 : -1;

        m_armVertical.set(ArmConstants.actuatorSpeed * direction);
    }

    public double GetArmLength(){
        double potentiometerOutput = verticalPotentiometer.get();
        System.out.println(potentiometerOutput);
        potentiometerOutput -= ArmConstants.potentiometerMinValue;
        potentiometerOutput /= (ArmConstants.potentiometerMaxValue - ArmConstants.potentiometerMinValue);

        double currentActuatorLength = 12 * potentiometerOutput;

        return currentActuatorLength;
    }

    public void StopArmVertical(){
        m_armVertical.stopMotor();
        
        autoRun = false;

        targetLength = GetArmLength();
    }

    public void setArmSpeed(double input) {
        m_armVertical.set(input * ArmConstants.actuatorSpeed);
        
        autoRun = false;
    }
}
