package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntake extends Command {

    private final AlgaeSubsystem m_algae;

    public AlgaeIntake(AlgaeSubsystem algae) {

        m_algae = algae;
        addRequirements(m_algae);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algae.setWheelMotors(0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_algae.stopWheelMotors();
    }
}
