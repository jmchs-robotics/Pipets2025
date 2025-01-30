package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class RotateOnce extends Command {

    private ElevatorSubsystem m_elevator;
    private double initialClicks;

    public RotateOnce(ElevatorSubsystem elevator) {

        m_elevator = elevator;
        addRequirements(m_elevator);

    }

    @Override
    public void initialize() {
        initialClicks = m_elevator.getPrimaryMotor().getPosition().getValueAsDouble();
    }

    @Override
    public void execute() {
        m_elevator.getPrimaryMotor().set(0.1);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_elevator.getPrimaryMotor().getPosition().getValueAsDouble()) >= (initialClicks + 1);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopMotors();
    }

    
}
