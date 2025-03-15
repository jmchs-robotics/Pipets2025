package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class LowerElevatorManual extends Command {

    private ElevatorSubsystem m_elevator;

    public LowerElevatorManual(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setElevatorManual(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopMotorsManual();
    }
    
}
