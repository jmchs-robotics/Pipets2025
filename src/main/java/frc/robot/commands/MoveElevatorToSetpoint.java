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
            case HOMED:
                m_elevator.setPositionInches(ElevatorPosition.HOMED.positionInches);
                break;
            case L2_ALGAE:
                m_elevator.setPositionInches(ElevatorPosition.L2_ALGAE.positionInches);
                break;
            case L3_ALGAE:
                m_elevator.setPositionInches(ElevatorPosition.L3_ALGAE.positionInches);
                break;
        }

        m_elevator.moveToSetpoint();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
