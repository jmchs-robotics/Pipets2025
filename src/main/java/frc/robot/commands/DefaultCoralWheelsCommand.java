package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWheelsSubsystem;

public class DefaultCoralWheelsCommand extends Command {

    private CoralWheelsSubsystem m_coralWheels;

    public DefaultCoralWheelsCommand(CoralWheelsSubsystem coralWheels) {
        m_coralWheels = coralWheels;
        addRequirements(m_coralWheels);
    }

    @Override
    public void execute() {
        m_coralWheels.setWheelMotors(-0.05);
    }
}
