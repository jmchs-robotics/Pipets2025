package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;

public class FlipCoralFlipperDown extends Command {
    
    private final CoralFlipperSubsystem m_coralFlipper;

    public FlipCoralFlipperDown(CoralFlipperSubsystem coralFlipper) {

        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_coralFlipper.flipCoralFlipperDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}