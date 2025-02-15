package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignToCoralStation extends Command {

    public DriveSubsystem m_drive;

    public AutoAlignToCoralStation(DriveSubsystem drive) {

        m_drive = drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        m_drive.pathFindToCoralStation();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
