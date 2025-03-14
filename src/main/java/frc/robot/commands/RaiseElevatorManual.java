package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RaiseElevatorManual extends Command {

    private ElevatorSubsystem m_elevator;

    public RaiseElevatorManual(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void execute() {
        m_elevator.setElevatorManual(0.2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopMotorsManual();
    }
    
}
