package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class RaiseElevator extends Command {

    private final ElevatorSubsystem m_elevator;

    public RaiseElevator(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        m_elevator.setMotors(0.5);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        m_elevator.stopMotors();

    }
    
}
