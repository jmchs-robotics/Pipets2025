package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.LEDSubsystem;

public class DefaultLEDCommand extends Command {

    private final LEDSubsystem m_led;

    private Alliance alliance = null;

    public DefaultLEDCommand(LEDSubsystem led) {
        m_led = led;
        addRequirements(m_led);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                alliance = Alliance.Blue;
            } else {
                alliance = Alliance.Red;
            }
        }
    }

    @Override
    public void execute() {
        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_4_CORAL) {
            LEDPattern pattern = LEDPattern.solid(Color.kWhite);
            m_led.setLEDPattern(pattern);
        }

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_CORAL) {
            LEDPattern pattern = LEDPattern.steps(
                Map.of(
                    0, 
                    Color.kWhite, 
                    2.0/3.0, 
                    alliance == Alliance.Blue 
                        ? Color.kBlue
                        : Color.kRed)
                );
            m_led.setLEDPattern(pattern);
        }

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_CORAL) {
            LEDPattern pattern = LEDPattern.steps(
                Map.of(
                    0,
                    Color.kWhite,
                    1.0/3.0,
                    alliance == Alliance.Blue
                        ? Color.kBlue
                        : Color.kRed
                )
            );
            m_led.setLEDPattern(pattern);
        }

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_ALGAE) {
            LEDPattern pattern = LEDPattern.solid(Color.kSeaGreen);
            m_led.setLEDPattern(pattern);
        }

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_ALGAE) {

            LEDPattern pattern = LEDPattern.steps(
                Map.of(
                    0,
                    Color.kSeaGreen,
                    1.0/2.0,
                    alliance == Alliance.Blue
                        ? Color.kBlue
                        : Color.kRed
                )
            );
            m_led.setLEDPattern(pattern);
        }
    }
}
