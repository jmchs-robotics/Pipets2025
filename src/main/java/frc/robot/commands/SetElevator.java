package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
    
    private final ElevatorSubsystem m_elevator;
    private final ElevatorLevel m_level;

    public SetElevator(ElevatorSubsystem elevator, ElevatorLevel level) {
        m_level = level;

        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {

        if (m_level == ElevatorLevel.LEVEL_4_CORAL) {
            m_elevator.setPosition(ElevatorConstants.L4_CORAL);
        }

        if (m_level == ElevatorLevel.LEVEL_3_CORAL) {
            m_elevator.setPosition(ElevatorConstants.L3_CORAL);
        }

        if (m_level == ElevatorLevel.LEVEL_3_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L3_ALGAE);
        }

        if (m_level == ElevatorLevel.LEVEL_2_CORAL) {
            m_elevator.setPosition(ElevatorConstants.L2_CORAL);
        }

        if (m_level == ElevatorLevel.LEVEL_2_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L2_ALGAE);
        }

        if (m_level == ElevatorLevel.HOME) {
            m_elevator.setPosition(ElevatorConstants.HOME);
        }

        if (m_level == ElevatorLevel.CORAL_STATION) {
            m_elevator.setPosition(ElevatorConstants.CORAL_STATION);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
