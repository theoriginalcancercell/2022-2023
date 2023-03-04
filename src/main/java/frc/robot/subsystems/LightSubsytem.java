// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LEDLightsConstants;

// public class LightSubsytem extends SubsystemBase {
//     AddressableLED leds = new AddressableLED(LEDLightsConstants.ledPort);
//     AddressableLEDBuffer ledBuffer;

//     public void InitializeLights(){
//         ledBuffer = new AddressableLEDBuffer(LEDLightsConstants.ledLength);
//         leds.setLength(ledBuffer.getLength());

//         leds.setData(ledBuffer);
//         leds.start();
//     }

//     //0 is off 1 is cube 2 is cone
//     public void ChangeLightState(int state){
//         Color ledColor = LEDLightsConstants.colors[state];

//         for(int i = 0; i < ledBuffer.getLength(); i++) {
//             ledBuffer.setRGB(i, (int)(ledColor.red * 255), (int)(ledColor.green * 255), (int)(ledColor.blue * 255));
//         }

//         leds.setData(ledBuffer);
//     }
// }
