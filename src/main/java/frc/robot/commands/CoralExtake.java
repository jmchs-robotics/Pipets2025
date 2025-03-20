package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralWheelsSubsystem;

public class CoralExtake extends Command {

    private final CoralWheelsSubsystem m_coralWheels;

    public CoralExtake(CoralWheelsSubsystem coralWheels) {

        m_coralWheels = coralWheels;
        addRequirements(m_coralWheels);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_coralWheels.setWheelMotors(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_coralWheels.stopWheelMotors();
    }
}