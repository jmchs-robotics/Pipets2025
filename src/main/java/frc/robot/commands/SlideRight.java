package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SlideRight extends Command {

    private final DriveSubsystem m_drive;

    public SlideRight(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
   }

    @Override
    public void execute() {
        m_drive.driveRobotRelative(new ChassisSpeeds(0, -0.1, 0));     
    }
}
