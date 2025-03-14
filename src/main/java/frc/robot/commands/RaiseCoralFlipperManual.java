package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;

public class RaiseCoralFlipperManual extends Command {

    private CoralFlipperSubsystem m_coralFlipper;

    public RaiseCoralFlipperManual(CoralFlipperSubsystem coralFlipper) {

        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);

    }

    @Override
    public void execute() {
        m_coralFlipper.setCoralFlipperManual(-0.1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_coralFlipper.stopMotorsManual();
    }
    
}
