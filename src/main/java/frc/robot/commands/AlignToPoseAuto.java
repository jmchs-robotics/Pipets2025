package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.ReefAlignment;
import frc.robot.RobotContainer.ReefSide;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignToPoseAuto extends Command {

    private final DriveSubsystem m_drive;
    private Pose2d goalPose;

    PIDController m_xController = new PIDController(0.75, 0, 0);
    PIDController m_yController = new PIDController(0.75, 0, 0);
    PIDController m_yawController = new PIDController(0.01, 0, 0);

    private final double xThreshold = Units.inchesToMeters(1); // meters
    private final double yThreshold = Units.inchesToMeters(1); // meters
    private final double yawThreshold = 2; // degrees

    private ReefSide side;
    private ReefAlignment alignment;
    private Alliance alliance;

    public AlignToPoseAuto(DriveSubsystem drive, char reefPoint) {

        m_yawController.enableContinuousInput(-180, 180);

        m_drive = drive;
        addRequirements(m_drive);

        switch (reefPoint) {
            case 'A':
                side = ReefSide.BACK_MIDDLE;
                alignment = ReefAlignment.LEFT;
                break;
            case 'B':
                side = ReefSide.BACK_MIDDLE;
                alignment = ReefAlignment.RIGHT;
                break;
            case 'C':
                side = ReefSide.BACK_RIGHT;
                alignment = ReefAlignment.LEFT;
                break;
            case 'D':
                side = ReefSide.BACK_RIGHT;
                alignment = ReefAlignment.RIGHT;
                break;
            case 'E':
                side = ReefSide.FRONT_RIGHT;
                alignment = ReefAlignment.LEFT;
                break;
            case 'F':
                side = ReefSide.FRONT_RIGHT;
                alignment = ReefAlignment.RIGHT;
                break;
            case 'G':
                side = ReefSide.FRONT_MIDDLE;
                alignment = ReefAlignment.LEFT;
                break;
            case 'H':
                side = ReefSide.FRONT_MIDDLE;
                alignment = ReefAlignment.RIGHT;
                break;
            case 'I':
                side = ReefSide.FRONT_LEFT;
                alignment = ReefAlignment.LEFT;
                break;
            case 'J':
                side = ReefSide.FRONT_LEFT;
                alignment = ReefAlignment.RIGHT;
                break;
            case 'K':
                side = ReefSide.BACK_LEFT;
                alignment = ReefAlignment.LEFT;
                break;
            case 'L':
                side = ReefSide.BACK_LEFT;
                alignment = ReefAlignment.RIGHT;
                break;
        }

    }

    @Override
    public void initialize() {

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

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
