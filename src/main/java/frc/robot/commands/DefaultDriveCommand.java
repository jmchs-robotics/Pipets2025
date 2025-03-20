package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DefaultDriveCommand extends Command {

    private final DriveSubsystem m_drive;
    private final CommandXboxController m_controller;

    public DefaultDriveCommand(DriveSubsystem drive, CommandXboxController controller) {

        m_controller = controller;

        m_drive = drive;
        addRequirements(m_drive);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drive.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY() * 0.9, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getLeftX() * 0.9, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getRightX() * 0.9, OIConstants.kDriveDeadband),
            true
        );

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
