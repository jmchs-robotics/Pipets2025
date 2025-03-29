package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;

public class ZeroCoralFlipper extends Command {
    
    private final CoralFlipperSubsystem m_coralFlipper;

    public ZeroCoralFlipper(CoralFlipperSubsystem coralFlipper) {

        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);

    }

    @Override
    public boolean isFinished() {
        return (m_coralFlipper.getCurrentPosition() < 0.05 && m_coralFlipper.getCurrentVelocity() == 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralFlipper.setNeutral();
        m_coralFlipper.setPosition(CoralConstants.kIdleAngle);
    }

}
