package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class DefaultAlgaeCommand extends Command {
    
    private final AlgaeSubsystem m_algae;

    public DefaultAlgaeCommand(AlgaeSubsystem algae) {

        m_algae = algae;
        addRequirements(m_algae);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algae.flipFlipperDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
