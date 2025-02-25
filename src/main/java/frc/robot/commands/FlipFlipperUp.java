package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class FlipFlipperUp extends Command {
    
    private final AlgaeSubsystem m_algae;

    public FlipFlipperUp(AlgaeSubsystem algae) {

        m_algae = algae;
        addRequirements(m_algae);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algae.flipFlipperUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
