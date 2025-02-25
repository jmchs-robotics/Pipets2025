package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeFlipperSubsystem;

public class DefaultAlgaeFlipperCommand extends Command {
    
    private final AlgaeFlipperSubsystem m_algaeFlipper;

    public DefaultAlgaeFlipperCommand(AlgaeFlipperSubsystem algaeFlipper) {

        m_algaeFlipper = algaeFlipper;
        addRequirements(m_algaeFlipper);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algaeFlipper.flipFlipperDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
