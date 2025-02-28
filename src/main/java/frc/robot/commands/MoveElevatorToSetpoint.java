package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;

public class MoveElevatorToSetpoint extends Command {
    
    private final ElevatorSubsystem m_elevator;

    public MoveElevatorToSetpoint(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {

        m_elevator.setCurrentTarget(ElevatorPosition.L4_CORAL);

        if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_4_CORAL) {
            m_elevator.setCurrentTarget(ElevatorPosition.L4_CORAL);
            m_elevator.setPositionInches(ElevatorPosition.L4_CORAL.positionInches);
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_CORAL) {
            m_elevator.setCurrentTarget(ElevatorPosition.L3_CORAL);
            m_elevator.setPositionInches(ElevatorPosition.L3_CORAL.positionInches);
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_3_ALGAE) {
            m_elevator.setCurrentTarget(ElevatorPosition.L3_ALGAE);
            m_elevator.setPositionInches(ElevatorPosition.L3_ALGAE.positionInches);
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_CORAL) {
            m_elevator.setCurrentTarget(ElevatorPosition.L2_CORAL);
            m_elevator.setPositionInches(ElevatorPosition.L2_CORAL.positionInches);
        } else if (RobotContainer.elevatorLevel == ElevatorLevel.LEVEL_2_ALGAE) {
            m_elevator.setCurrentTarget(ElevatorPosition.L2_ALGAE);
            m_elevator.setPositionInches(ElevatorPosition.L2_ALGAE.positionInches);
        }

    }

    @Override
    public void execute() {

        m_elevator.moveToSetpoint();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
