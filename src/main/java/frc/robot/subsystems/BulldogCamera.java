package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class BulldogCamera {
    
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private final String name;
    private final PhotonCamera cam;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator photonPoseEstimator;

    public Pose2d camPose;
    public double camTimestamp;

    public BulldogCamera(String name, Transform3d robotToCam) {

        this.name = name;
        cam = new PhotonCamera(name);
        this.robotToCam = robotToCam;

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public void updateVision() {

        var results = cam.getAllUnreadResults();
        if (!results.isEmpty()) {

            var result = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> currentPose = photonPoseEstimator.update(result);
    
            if (currentPose.isPresent()) {
    
                camPose = new Pose2d(
                    currentPose.get().estimatedPose.getX(),
                    currentPose.get().estimatedPose.getY(),
                    currentPose.get().estimatedPose.getRotation().toRotation2d()
                );
    
                camTimestamp = currentPose.get().timestampSeconds;
    
            }

        }

    }

    public String getName() {
        return name;
    }

    public void setReferencePose(Pose2d pose) {
        photonPoseEstimator.setReferencePose(pose);
    }
}
