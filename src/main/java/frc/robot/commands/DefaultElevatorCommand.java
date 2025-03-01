package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class DefaultElevatorCommand extends Command {

    private final ElevatorSubsystem m_elevator;

    public DefaultElevatorCommand(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (!m_elevator.isHomed()) {
            m_elevator.homeElevator();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
