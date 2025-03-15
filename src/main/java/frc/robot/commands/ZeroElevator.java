package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends Command {

    private final ElevatorSubsystem m_elevator;

    public ZeroElevator(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return (m_elevator.getCurrentPosition() < 1.5 && m_elevator.getCurrentVelocity() == 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setNeutral();
        m_elevator.resetSensorPosition(ElevatorConstants.HOME);
    }
}
