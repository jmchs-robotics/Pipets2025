package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.ReefAlignment;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.BulldogCamera;

public class AlignToReef extends Command {

    private final DriveSubsystem m_drive;
    private final BulldogCamera m_cam;
    private boolean isLeftSide;

    PIDController m_aimController = new PIDController(0.1, 0, 0);
    PIDController m_rangeController = new PIDController(0.1, 0, 0);
    PIDController m_strafeController = new PIDController(0.1, 0, 0);

    private double m_rangeTarget;
    private double m_strafeTarget;
    private double m_aimTarget;

    public AlignToReef(DriveSubsystem drive, BulldogCamera cam) {

        m_drive = drive;
        addRequirements(m_drive);

        m_cam = cam;

    }

    @Override
    public void initialize() {
        if (RobotContainer.reefAlignment == ReefAlignment.LEFT) {
            isLeftSide = true;
        } else if (RobotContainer.reefAlignment == ReefAlignment.RIGHT) {
            isLeftSide = false;
        }

        if (isLeftSide) {
            m_rangeTarget = DriveConstants.leftReefRangeTarget;
            m_strafeTarget = DriveConstants.leftReefStrafeTarget;
            m_aimTarget = DriveConstants.leftReefAimTarget;
        } else {
            m_rangeTarget = DriveConstants.rightReefRangeTarget;
            m_strafeTarget = DriveConstants.rightReefStrafeTarget;
            m_aimTarget = DriveConstants.rightReefAimTarget;
        }
    }

    @Override
    public void execute() {

        m_drive.driveRobotRelative(
            new ChassisSpeeds(getRangePID(), getStrafePID(), getAimPID())
        );

    }

    public double getAimPID() {
        
        double aimVal = m_aimController.calculate(m_cam.camToTagYaw, m_aimTarget);
        aimVal *= DriveConstants.kMaxAngularSpeed;

        return aimVal;

    }

    public double getRangePID() {
        
        double rangeVal = m_rangeController.calculate(m_cam.camToTagX, m_rangeTarget);
        rangeVal *= DriveConstants.kMaxSpeedMetersPerSecond;

        return rangeVal;

    }

    public double getStrafePID() {
        
        double strafeVal = m_strafeController.calculate(m_cam.camToTagY, m_strafeTarget);
        strafeVal *= DriveConstants.kMaxSpeedMetersPerSecond;

        return strafeVal;

    }
    
}
