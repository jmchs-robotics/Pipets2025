package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeFlipperSubsystem;

public class ZeroAlgaeFlipper extends Command {
    
    private final AlgaeFlipperSubsystem m_algaeFlipper;

    public ZeroAlgaeFlipper(AlgaeFlipperSubsystem algaeFlipper) {

        m_algaeFlipper = algaeFlipper;
        addRequirements(m_algaeFlipper);

    }

    @Override
    public boolean isFinished() {
        return (m_algaeFlipper.getCurrentPosition() > -0.05 && m_algaeFlipper.getCurrentVelocity() == 0);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_algaeFlipper.setNeutral();
            m_algaeFlipper.setPosition(AlgaeConstants.kAngleDown);
        }
    }

}
