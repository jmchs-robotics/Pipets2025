package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class BulldogCamera {
    
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private final String name;
    private final PhotonCamera cam;
    private List<PhotonTrackedTarget> targets;
    private final PhotonPoseEstimator photonPoseEstimator;

    public Pose2d camPose;
    public double camTimestamp;
    public double minDistance;

    private final int[] validTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public BulldogCamera(String name, Transform3d robotToCam) {

        this.name = name;
        cam = new PhotonCamera(name);

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public void updateVision() {

        var results = cam.getAllUnreadResults();
        if (!results.isEmpty()) {

            var rawResult = results.get(results.size() - 1);
            var targetsUsed = rawResult.targets;
            var filteredTargets = filterTags(targetsUsed);
            // the getMultiTagresult might mess with the tag filtering if it includes filtered out tags
            // the metadata might also goof with the filtered tags
            var newResult = new PhotonPipelineResult(
                rawResult.metadata,
                filteredTargets,
                rawResult.getMultiTagResult()
            );
            Optional<EstimatedRobotPose> currentPose = photonPoseEstimator.update(newResult);
    
            if (currentPose.isPresent()) {

                if (newResult.hasTargets()) {
                    targets = currentPose.get().targetsUsed;
                }

                if (targets != null) {

                    if (targets.size() > 1) {
                        double minDistance = Double.MAX_VALUE;
                        for (PhotonTrackedTarget target : targets) {
                            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                            if (distance < minDistance) {
                                minDistance = distance;
                            }
                        }
                        this.minDistance = minDistance;
                    } else {
                        minDistance = targets.get(0).getBestCameraToTarget().getTranslation().getNorm();
                    }

                }
    
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

    public double getMinDistance() {
        return minDistance;
    }

    private List<PhotonTrackedTarget> filterTags(List<PhotonTrackedTarget> tags) {

        List<PhotonTrackedTarget> filteredResults = tags;

        for (int i = filteredResults.size() - 1; i >= 0; i--) {
            if (!isValidTag(filteredResults.get(i).fiducialId)) {
                filteredResults.remove(i);
            }
        }

        return filteredResults;

    }

    private boolean isValidTag(int tagNum) {
        for (int tag : validTags) {
            if (tagNum == tag) {
                return true;
            }
        }
        return false;
    }

}
