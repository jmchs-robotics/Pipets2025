// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.ReefAlignment;
import frc.robot.RobotContainer.ReefSide;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final MAXSwerveModule[] mSwerveModules = {
    m_frontLeft,
    m_frontRight,
    m_rearLeft,
    m_rearRight
  };

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_estimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
  );

  // private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  // private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  private final VisionSubsystem vision;
  private RobotConfig robotConfig;

  @Logged(name = "Estimated Pose", importance = Importance.INFO)
  private Pose2d estimatedPose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(VisionSubsystem vision) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    this.vision = vision;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    
    configPathPlanner();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_estimator.update(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    // for (int i = 0; i < vision.getCameraPoses().size(); i++) {
    //   m_estimator.addVisionMeasurement(
    //     vision.getCameraPoses().get(i),
    //     vision.getCameraTimestamps().get(i),
    //     VecBuilder.fill(
    //       vision.getMinDistance(i) * DriveConstants.kEstimationCoefficient, 
    //       vision.getMinDistance(i) * DriveConstants.kEstimationCoefficient,
    //       5.0
    //     )
    //   );
    // }

    SmartDashboard.putNumber("Gyro Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());

    vision.setReferencePose(getPose());
    estimatedPose = getPose();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_estimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_estimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] currentStates = new SwerveModuleState[mSwerveModules.length];
    for (int i = 0; i < mSwerveModules.length; i++){
      currentStates[i] = mSwerveModules[i].getState();
    }
    return currentStates;      
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < mSwerveModules.length; i++) {
      mSwerveModules[i].setDesiredState(targetStates[i]);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void configPathPlanner() {

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), 
      this::driveRobotRelative,
      DriveConstants.ppDriveController,
      robotConfig, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this
    );
  }

  public RobotConfig getRobotConfig() {
    return robotConfig;
  }

  // public void pathFindToReef() {}

  public Command pathFindToProcessor() {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(6.042, 1.490, Rotation2d.fromDegrees(-90)),
      new Pose2d(6.042, 0.519, Rotation2d.fromDegrees(-90))
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      DriveConstants.constraints, 
      null, 
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
    );

    return AutoBuilder.followPath(path);

  }

  public Command pathFindToCoralStationRight() {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(2.218, 1.478, Rotation2d.fromDegrees(-127.500)),
      new Pose2d(1.618,0.674, Rotation2d.fromDegrees(-127.500))
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      DriveConstants.constraints, 
      null, 
      new GoalEndState(0.0, Rotation2d.fromDegrees(-127.5)));
    
    return AutoBuilder.followPath(path);

  }

  public Command pathFindToCoralStationLeft() {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(2.098, 6.680, Rotation2d.fromDegrees(127.500)),
      new Pose2d(1.606,7.328, Rotation2d.fromDegrees(127.500))
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      DriveConstants.constraints, 
      null, 
      new GoalEndState(0.0, Rotation2d.fromDegrees(127.5)));
    
    return AutoBuilder.followPath(path);

  }

  public Command pathFindToReef() {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d());
    GoalEndState endState = new GoalEndState(0, Rotation2d.fromDegrees(0));

    if (RobotContainer.reefAlignment == ReefAlignment.LEFT) {
      if (RobotContainer.reefSide == ReefSide.FRONT_LEFT) {
        waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(5.577, 6.014, Rotation2d.fromDegrees(-120)),
            new Pose2d(5.195, 5.297, Rotation2d.fromDegrees(-120))
          );
          
          endState = new GoalEndState(0, Rotation2d.fromDegrees(-120));
          
        }
        else if (RobotContainer.reefSide == ReefSide.FRONT_MIDDLE) {
          waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(6.655, 4.019, Rotation2d.fromDegrees(180)),
            new Pose2d(5.971, 4.047, Rotation2d.fromDegrees(180))
          );

          endState = new GoalEndState(0, Rotation2d.fromDegrees(180));

        }
        else if (RobotContainer.reefSide == ReefSide.FRONT_RIGHT) {
          waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(5.616, 2.075, Rotation2d.fromDegrees(120)),
            new Pose2d(5.227, 2.735, Rotation2d.fromDegrees(120))
          );
          
          endState = new GoalEndState(0, Rotation2d.fromDegrees(120));
          
        }
        else if (RobotContainer.reefSide == ReefSide.BACK_LEFT) {
          waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(3.413, 5.829, Rotation2d.fromDegrees(-60)),
            new Pose2d(3.753, 5.355, Rotation2d.fromDegrees(-60))
          );

          endState = new GoalEndState(0, Rotation2d.fromDegrees(-60));
          
        }
        else if (RobotContainer.reefSide == ReefSide.BACK_MIDDLE) {
          waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.457, 4.005, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 4, Rotation2d.fromDegrees(0))
          );
          
          endState = new GoalEndState(0, Rotation2d.fromDegrees(0));
          
        }
        else if (RobotContainer.reefSide == ReefSide.BACK_RIGHT) {
          waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(3.354, 2.075, Rotation2d.fromDegrees(60)),
            new Pose2d(3.785, 2.723, Rotation2d.fromDegrees(60))
          );
          
          endState = new GoalEndState(0, Rotation2d.fromDegrees(60));
          
        }

    } else if (RobotContainer.reefAlignment == ReefAlignment.RIGHT) {
      if (RobotContainer.reefSide == ReefSide.FRONT_LEFT) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(5.255, 6.033, Rotation2d.fromDegrees(-120)),
          new Pose2d(4.895, 5.477, Rotation2d.fromDegrees(-120))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(-120));
          
      }
      else if (RobotContainer.reefSide == ReefSide.FRONT_MIDDLE) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(6.728, 4.366, Rotation2d.fromDegrees(180)),
          new Pose2d(5.971, 4.387, Rotation2d.fromDegrees(180))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(180));
          
      }
      else if (RobotContainer.reefSide == ReefSide.FRONT_RIGHT) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(5.899, 2.345, Rotation2d.fromDegrees(120)),
          new Pose2d(5.527, 2.915, Rotation2d.fromDegrees(120))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(120));
          
      }
      else if (RobotContainer.reefSide == ReefSide.BACK_LEFT) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(3.058, 5.721, Rotation2d.fromDegrees(-60)),
          new Pose2d(3.453, 5.165, Rotation2d.fromDegrees(-60))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(-60));
          
      }
      else if (RobotContainer.reefSide == ReefSide.BACK_MIDDLE) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(2.320, 3.684, Rotation2d.fromDegrees(0)),
          new Pose2d(3, 3.643, Rotation2d.fromDegrees(0))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(0));
          
      }
      else if (RobotContainer.reefSide == ReefSide.BACK_RIGHT) {
        waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(3.673, 1.938, Rotation2d.fromDegrees(60)),
          new Pose2d(4.085, 2.533, Rotation2d.fromDegrees(60))
        );
        
        endState = new GoalEndState(0, Rotation2d.fromDegrees(60));
          
      }
    }


    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      DriveConstants.constraints, 
      null, 
      endState);
    
    return AutoBuilder.followPath(path);

  }

}
