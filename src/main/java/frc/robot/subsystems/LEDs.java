package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs {
    m_led = new AddressableLED(9);


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Apply the LED pattern to the data buffer
    gradient.applyTo(m_ledBuffer);

    // Write the data to the LED strip

    m_led.setData(m_ledBuffer);
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {// TODO alliance is red idk how to write it
        m_led.setData(m_ledBuffer);
        m_led.start();

        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,
        Color.kRed, Color.kPink, Color.kCrimson, Color.kSienna, Color.kTomato, Color.kDarkOrange, 
        Color.kDarkMagenta, Color.kFirebrick);}
        
    // blue alliance color scheme
    else () {
        m_led.setData(m_ledBuffer);
        m_led.start();

        LEDParrern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, 
        Color.kBlue, Color.kBlue, Color.kAliceBlue, Color.kAqua, Color.kAquamarine, Color.kAzure, 
        Color.kBlueViolet, Color.kCadetBlue, Color.kDarkBlue, Color.kDarkBlue, Color.kDarkCyan, Color.kMediumPurple)

    }

}
