package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;

public class DefaultCoralFlipperCommand extends Command {
    
    private final CoralFlipperSubsystem m_coralFliipper;

    public DefaultCoralFlipperCommand(CoralFlipperSubsystem coralFlipper) {

        m_coralFliipper = coralFlipper;
        addRequirements(m_coralFliipper);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_coralFliipper.flipCoralFlipperUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}