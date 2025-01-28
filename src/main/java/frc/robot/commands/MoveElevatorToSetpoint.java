package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class MoveElevatorToSetpoint extends Command {
    
    private final ElevatorSubsystem m_elevator;

    public MoveElevatorToSetpoint(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        switch (m_elevator.getCurrentTarget()) {
            case DOWN:
                m_elevator.setPositionInches(ElevatorPosition.DOWN.positionInches);
                break;
            case POSITION_1:
                m_elevator.setPositionInches(ElevatorPosition.POSITION_1.positionInches);
                break;
            case POSITION_2:
                m_elevator.setPositionInches(ElevatorPosition.POSITION_2.positionInches);
                break;
            case POSITION_3:
                m_elevator.setPositionInches(ElevatorPosition.POSITION_3.positionInches);
                break;
            case POSITION_4:
                m_elevator.setPositionInches(ElevatorPosition.POSITION_4.positionInches);
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        m_elevator.setPositionInches(ElevatorPosition.DOWN.positionInches);

    }
    
}
