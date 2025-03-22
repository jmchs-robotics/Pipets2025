package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignToPose extends Command {

    private final DriveSubsystem m_drive;
    private final Pose2d m_pose;

    PIDController m_xController = new PIDController(0.5, 0, 0);
    PIDController m_yController = new PIDController(0.5, 0, 0);
    PIDController m_yawController = new PIDController(0.01, 0, 0);

    private final double xThreshold = 0.3; // meters
    private final double yThreshold = 0.3; // meters
    private final double yawThreshold = 10; // degrees

    public AlignToPose(DriveSubsystem drive, Pose2d pose) {

        m_drive = drive;
        addRequirements(m_drive);

        m_pose = pose;

    }

    @Override
    public void execute() {

        m_drive.driveRobotRelative(
            new ChassisSpeeds(getXPID(), getYPID(), getYawPID())
        );

    }

    public double getXPID() {
        
        double xVal = m_xController.calculate(m_drive.getPose().getX(), m_pose.getX());
        xVal = MathUtil.clamp(xVal, -1, 1);

        xVal *= DriveConstants.kMaxSpeedMetersPerSecond;

        return xVal;

    }

    public double getYPID() {
        
        double yVal = m_yController.calculate(m_drive.getPose().getY(), m_pose.getY());
        yVal = MathUtil.clamp(yVal, -1, 1);

        yVal *= DriveConstants.kMaxSpeedMetersPerSecond;

        return yVal;

    }

    public double getYawPID() {
        
        double yawVal = m_yawController.calculate(m_drive.getPose().getRotation().getDegrees(), m_pose.getRotation().getDegrees());
        yawVal = MathUtil.clamp(yawVal, -1, 1);

        yawVal *= DriveConstants.kMaxAngularSpeed;

        return yawVal;

    }    

    @Override
    public boolean isFinished() {
        return (
            Math.abs(m_drive.getPose().getX() - m_pose.getX()) < xThreshold &&
            Math.abs(m_drive.getPose().getY() - m_pose.getY()) < yThreshold &&
            Math.abs(m_drive.getPose().getRotation().getDegrees() - m_pose.getRotation().getDegrees()) < yawThreshold
        );

    }
    
}
