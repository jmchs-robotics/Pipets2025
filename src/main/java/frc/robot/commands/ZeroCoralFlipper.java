package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;

public class ZeroCoralFlipper extends Command {
    
    private final CoralFlipperSubsystem m_coralFlipper;
    private boolean isFinished;

    public ZeroCoralFlipper(CoralFlipperSubsystem coralFlipper) {

        m_coralFlipper = coralFlipper;
        addRequirements(m_coralFlipper);

    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if (m_coralFlipper.getCurrentPosition() < 0.05) {
            m_coralFlipper.setCoralFlipperManual(-0.1);

            if (m_coralFlipper.getSupplyCurrent() > 5) {
                isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_coralFlipper.setNeutral();
        m_coralFlipper.setPosition(CoralConstants.kIdleAngle);
    }

}
