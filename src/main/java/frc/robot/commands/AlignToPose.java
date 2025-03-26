package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ReefAlignment;
import frc.robot.RobotContainer.ReefSide;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignToPose extends Command {

    private final DriveSubsystem m_drive;
    private Pose2d goalPose;

    PIDController m_xController = new PIDController(0.75, 0, 0);
    PIDController m_yController = new PIDController(0.75, 0, 0);
    PIDController m_yawController = new PIDController(0.01, 0, 0);

    private final double xThreshold = 0.05; // meters
    private final double yThreshold = 0.05; // meters
    private final double yawThreshold = 5; // degrees

    private ReefSide side;
    private ReefAlignment alignment;
    private Alliance alliance;

    public AlignToPose(DriveSubsystem drive) {

        m_yawController.enableContinuousInput(-180, 180);

        m_drive = drive;
        addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        side = RobotContainer.reefSide;
        alignment = RobotContainer.reefAlignment;
        
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        SmartDashboard.putString("Reef Side Align", side.toString());
        SmartDashboard.putString("Reef Alignment Align", alignment.toString());
        SmartDashboard.putString("Alliance Align", alliance.toString());

        determinePose();
    }

    @Override
    public void execute() {

        m_drive.drive(-getXPID(), -getYPID(), getYawPID(), true);

    }

    @Override
    public boolean isFinished() {
        return (
            Math.abs(m_drive.getPose().getX() - goalPose.getX()) < xThreshold &&
            Math.abs(m_drive.getPose().getY() - goalPose.getY()) < yThreshold &&
            Math.abs(m_drive.getPose().getRotation().getDegrees() - goalPose.getRotation().getDegrees()) < yawThreshold
        );

    }

    private double getXPID() {
        
        double xVal = m_xController.calculate(m_drive.getPose().getX(), goalPose.getX());
        xVal = MathUtil.clamp(xVal, -1, 1);

        return xVal;

    }

    private double getYPID() {
        
        double yVal = m_yController.calculate(m_drive.getPose().getY(), goalPose.getY());
        yVal = MathUtil.clamp(yVal, -1, 1);

        return yVal;

    }

    private double getYawPID() {
        
        double yawVal = m_yawController.calculate(m_drive.getPose().getRotation().getDegrees(), goalPose.getRotation().getDegrees());
        yawVal = MathUtil.clamp(yawVal, -1, 1);

        return yawVal;

    }

    private void determinePose() {

        // Default is Blue G (Front Middle Left)
        goalPose = new Pose2d(5.91, 3.9, Rotation2d.fromDegrees(180));
        Translation2d blueReefCenter = new Translation2d(4.489, 4.025);

        if (alignment == ReefAlignment.RIGHT) {
            goalPose = new Pose2d(5.91, 4.28, Rotation2d.fromDegrees(180));
        }

        switch (side) {
            case FRONT_LEFT:
                goalPose = goalPose.rotateAround(blueReefCenter, Rotation2d.fromDegrees(60));
                break;
            case FRONT_MIDDLE:
                // already front middle, don't do anything
                break;
            case FRONT_RIGHT:
                goalPose = goalPose.rotateAround(blueReefCenter, Rotation2d.fromDegrees(-60));
                break;
            case BACK_LEFT:
                goalPose = goalPose.rotateAround(blueReefCenter, Rotation2d.fromDegrees(120));
                break;
            case BACK_MIDDLE:
                goalPose = goalPose.rotateAround(blueReefCenter, Rotation2d.fromDegrees(180));
                break;
            case BACK_RIGHT:
                goalPose = goalPose.rotateAround(blueReefCenter, Rotation2d.fromDegrees(-120));
                break;
        }

        if (alliance == Alliance.Red) {
            goalPose = goalPose.rotateAround(new Translation2d(8.775, 4.025), Rotation2d.fromDegrees(180));
        }

    }
    
}
