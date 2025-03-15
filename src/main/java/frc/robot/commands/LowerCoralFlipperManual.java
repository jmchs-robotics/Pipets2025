package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFlipperSubsystem;

public class LowerCoralFlipperManual extends Command {

    private CoralFlipperSubsystem m_coralFlipper;

    public LowerCoralFlipperManual(CoralFlipperSubsystem coralFlipper) {
        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);
    }

    @Override
    public void execute() {
        m_coralFlipper.setCoralFlipperManual(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralFlipper.stopMotorsManual();
    }
    
}
