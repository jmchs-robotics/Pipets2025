package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevator extends Command {
    
    private final ElevatorSubsystem m_elevator;
    private String m_level;

    public SetElevator(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

        m_level = "L4_CORAL";

    }

    @Override
    public void initialize() {

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_4_CORAL) {
            m_level = "L4_CORAL";
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_CORAL) {
            m_level = "L3_CORAL";
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_ALGAE) {
            m_level = "L3_ALGAE";
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_CORAL) {
            m_level = "L2_CORAL";
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_ALGAE) {
            m_level = "L2_ALGAE";
        }

    }

    @Override
    public void execute() {

        if (m_level.equals("L4_CORAL")) {
            m_elevator.setPosition(ElevatorConstants.L4_CORAL);
        }

        if (m_level.equals("L3_CORAL")) {
            m_elevator.setPosition(ElevatorConstants.L3_CORAL);
        }

        if (m_level.equals("L3_ALGAE")) {
            m_elevator.setPosition(ElevatorConstants.L3_ALGAE);
        }

        if (m_level.equals("L2_CORAL")) {
            m_elevator.setPosition(ElevatorConstants.L2_CORAL);
        }

        if (m_level.equals("L2_ALGAE")) {
            m_elevator.setPosition(ElevatorConstants.L2_ALGAE);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
