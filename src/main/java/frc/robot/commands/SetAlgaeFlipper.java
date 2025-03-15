package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeFlipperSubsystem;

public class SetAlgaeFlipper extends Command {
    
    private final AlgaeFlipperSubsystem m_algaeFlipper;
    private final String position;

    public SetAlgaeFlipper(AlgaeFlipperSubsystem algaeFlipper, String pos) {
        position = pos;

        m_algaeFlipper = algaeFlipper;
        addRequirements(m_algaeFlipper); 
    }

    @Override
    public void execute() {

        if (position.equals("up")) {
            m_algaeFlipper.setPosition(AlgaeConstants.kAngleUp);
        }

        if (position.equals("down")) {
            m_algaeFlipper.setPosition(AlgaeConstants.kAngleDown);
        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}