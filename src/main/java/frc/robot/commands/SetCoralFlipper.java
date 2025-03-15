package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralFlipperSubsystem;

public class SetCoralFlipper extends Command {
    
    private final CoralFlipperSubsystem m_coralFlipper;
    private final String position;

    public SetCoralFlipper(CoralFlipperSubsystem coralFlipper, String pos) {
        position = pos;

        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);
    }

    @Override
    public void execute() {

        if (position.equals("idle")) {
            m_coralFlipper.setPosition(CoralConstants.kIdleAngle);
        }

        if (position.equals("scoreLow")) {
            m_coralFlipper.setPosition(CoralConstants.kScoreLow);
        }

        if (position.equals("scoreHigh")) {
            m_coralFlipper.setPosition(CoralConstants.kScoreHigh);
        }

        if (position.equals("coralStation")) {
            m_coralFlipper.setPosition(CoralConstants.kCoralStation);
        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}