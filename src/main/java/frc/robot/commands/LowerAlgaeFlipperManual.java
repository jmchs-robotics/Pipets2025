package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeFlipperSubsystem;

public class LowerAlgaeFlipperManual extends Command {

    private AlgaeFlipperSubsystem m_algaeFlipper;

    public LowerAlgaeFlipperManual(AlgaeFlipperSubsystem algaeFlipper) {
        m_algaeFlipper = algaeFlipper;
        addRequirements(m_algaeFlipper);
    }

    @Override
    public void execute() {
        m_algaeFlipper.setAlgaeFlipperManual(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeFlipper.stopMotorsManual();
    }
    
}
