// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LEDLightsConstants;

public class LightSubsytem extends SubsystemBase {
    //Gets the LED lights from their port
    AddressableLED leds = new AddressableLED(LEDLightsConstants.ledPort);
    AddressableLEDBuffer ledBuffer;

    public void InitializeLights(){
        //Initializes the lights and gets how many lights there are
        ledBuffer = new AddressableLEDBuffer(LEDLightsConstants.ledLength);
        leds.setLength(ledBuffer.getLength());

        //Starts the lights
        leds.setData(ledBuffer);
        leds.start();
    }

    //0 is off 1 is cube (blue) 2 is cone (yellow)
    public void ChangeLightState(int state){
        //Gets the desired color as a color variable
        Color ledColor = LEDLightsConstants.colors[state];

        //Sets each light to the desired color
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            //Here we have to convert from the color variable to raw rgb values
            ledBuffer.setRGB(i, (int)(ledColor.red * 255), (int)(ledColor.green * 255), (int)(ledColor.blue * 255));
        }

        //Sets the leds to their new values
        leds.setData(ledBuffer);
    }
}
