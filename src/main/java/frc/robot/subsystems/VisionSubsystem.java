package frc.robot.subsystems;

import java.security.Timestamp;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    BulldogCamera[] cam = {
        new BulldogCamera(new PhotonCamera("1"), new Transform3d()),
        new BulldogCamera(new PhotonCamera("2"), new Transform3d())
    };

    Pose2d camPose1 = new Pose2d (
        cam[0].getEstimatedGlobalPose().get().estimatedPose.getX(),
        cam[0].getEstimatedGlobalPose().get().estimatedPose.getY(),
        new Rotation2d(cam[0].getEstimatedGlobalPose().get().estimatedPose.getRotation().getAngle())
    );

    Pose2d camPose2 = new Pose2d (
        cam[1].getEstimatedGlobalPose().get().estimatedPose.getX(),
        cam[1].getEstimatedGlobalPose().get().estimatedPose.getY(),
        new Rotation2d(cam[1].getEstimatedGlobalPose().get().estimatedPose.getRotation().getAngle())
    );

    public Pose2d[] poseArray =  {camPose1, camPose2};
    public double[] timeArray =  {
        cam[0].getEstimatedGlobalPose().get().timestampSeconds,
        cam[1].getEstimatedGlobalPose().get().timestampSeconds
        };
}
