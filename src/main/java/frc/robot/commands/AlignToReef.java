package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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

    PIDController m_aimController = new PIDController(0.01, 0, 0);
    PIDController m_rangeController = new PIDController(1.0, 0, 0);
    PIDController m_strafeController = new PIDController(1.0, 0, 0);

    private double m_rangeTarget;
    private double m_strafeTarget;
    private double m_aimTarget;

    private final double rangeThreshold = 0.05; // meters
    private final double strafeThreshold = 0.05; // meters
    private final double aimThreshold = 1.0; //degrees

    // i have no idea what im doing - ian has no clue who wrote this
    // y'all please work on your problem solving skills - ian

    public AlignToReef(DriveSubsystem drive, BulldogCamera cam) {

        m_drive = drive;
        addRequirements(m_drive);

        m_cam = cam;

        // private var result = m_cam.getLatestResult();
        // private final PhotonTrackedTarget camtarget =  result.getBestTarget();
        // i'm going to blow up whoever wrote this - ian

        m_aimController.enableContinuousInput(-180, 180);

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
            new ChassisSpeeds(-getRangePID(), getStrafePID(), getAimPID())
        );

    }

    public double getAimPID() {
        
        double aimVal = m_aimController.calculate(m_cam.camToTagYaw, m_aimTarget);
        aimVal = MathUtil.clamp(aimVal, -1, 1);

        aimVal *= DriveConstants.kMaxAngularSpeed;

        return aimVal;

    }

    public double getRangePID() {
        
        double rangeVal = m_rangeController.calculate(m_cam.camToTagX, m_rangeTarget);
        rangeVal = MathUtil.clamp(rangeVal, -1, 1);

        rangeVal *= 0.25 * DriveConstants.kMaxSpeedMetersPerSecond;

        return rangeVal;

    }

    public double getStrafePID() {
        
        double strafeVal = m_strafeController.calculate(m_cam.camToTagY, m_strafeTarget);
        strafeVal = MathUtil.clamp(strafeVal, -1, 1);

        strafeVal *= DriveConstants.kMaxSpeedMetersPerSecond;

        return strafeVal;

    }

    @Override
    public boolean isFinished() {
        return (
            Math.abs(m_cam.camToTagY - m_strafeTarget) < strafeThreshold &&
            Math.abs(m_cam.camToTagX - m_rangeTarget) < rangeThreshold &&
            Math.abs(m_cam.camToTagYaw - m_aimTarget) < aimThreshold
        );

    }
    
}
