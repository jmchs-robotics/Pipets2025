package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeWheelsSubsystem;

public class AlgaeIntake extends Command {

    private final AlgaeWheelsSubsystem m_algaeWheels;

    public AlgaeIntake(AlgaeWheelsSubsystem algaeWheels) {
        m_algaeWheels = algaeWheels;
        addRequirements(m_algaeWheels);
    }

    @Override
    public void execute() {
        m_algaeWheels.setWheelMotors(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeWheels.stopWheelMotors();
    }
}
