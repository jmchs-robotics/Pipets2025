package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class HomeElevatorManually extends Command {

    private ElevatorSubsystem m_elevator;

    public HomeElevatorManually(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void execute() {
        m_elevator.setElevatorManual(-0.1);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.getSupplyCurrent() > 5;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setNeutral();
        m_elevator.resetSensorPosition(ElevatorConstants.HOME);
    }
    
}
