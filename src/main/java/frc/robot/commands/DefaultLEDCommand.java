package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class DefaultLEDCommand extends Command {

    private final LEDSubsystem m_led;

    public DefaultLEDCommand(LEDSubsystem led) {

        m_led = led;
        addRequirements(m_led);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (DriverStation.getAlliance().isPresent()) {

            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

                LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,
                Color.kRed, Color.kPink, Color.kCrimson, Color.kSienna, Color.kTomato, Color.kDarkOrange, 
                Color.kDarkMagenta, Color.kFirebrick);

                m_led.setLEDPattern(gradient);

            } else {

                LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, 
                Color.kBlue, Color.kAliceBlue, Color.kAqua, Color.kAquamarine, Color.kAzure, 
                Color.kBlueViolet, Color.kCadetBlue, Color.kDarkBlue, Color.kDarkCyan, Color.kMediumPurple);

                m_led.setLEDPattern(gradient);

            }

        } else {

            LEDPattern rainbow = LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 60.0));

            m_led.setLEDPattern(rainbow);

        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
