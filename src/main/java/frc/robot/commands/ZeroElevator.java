package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ZeroElevator extends Command {

    private final ElevatorSubsystem m_elevator;
    private boolean isFinished;

    public ZeroElevator(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if (m_elevator.getCurrentPosition() < 0.1) {
            m_elevator.setElevatorManual(-0.1);

            if (m_elevator.getSupplyCurrent() > 5) {
                isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_elevator.setNeutral();
            m_elevator.resetSensorPosition(ElevatorConstants.HOME);
        }
    }
    
}
