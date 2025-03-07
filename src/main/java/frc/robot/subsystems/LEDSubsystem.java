package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);

    public LEDSubsystem() {

        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();        

    }

    public void setLEDPattern(LEDPattern pattern) {

        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);

    }

    public void setScrollingRainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 60.0));

            setLEDPattern(rainbow);
    }

}

    
    