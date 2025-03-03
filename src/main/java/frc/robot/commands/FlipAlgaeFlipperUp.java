package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeFlipperSubsystem;

public class FlipAlgaeFlipperUp extends Command {
    
    private final AlgaeFlipperSubsystem m_algaeFlipper;

    public FlipAlgaeFlipperUp(AlgaeFlipperSubsystem algaeFlipper) {

        m_algaeFlipper = algaeFlipper;
        addRequirements(m_algaeFlipper);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algaeFlipper.flipFlipperUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
