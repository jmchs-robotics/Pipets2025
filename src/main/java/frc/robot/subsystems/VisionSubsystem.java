package frc.robot.subsystems;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private BulldogCamera[] cams = {
        new BulldogCamera("1", new Transform3d()),
        new BulldogCamera("2", new Transform3d())
    };

    private List<Pose2d> camPoses = new ArrayList<Pose2d>();
    private List<Double> camTimestamps = new ArrayList<Double>();

    public VisionSubsystem() {}

    @Override
    public void periodic() {

        camPoses.clear();
        camTimestamps.clear();

        for (int i = 0; i < cams.length; i++) {

            cams[i].updateVision();
            processCamera(i);

        }

    }

    public void processCamera(int camNum) {

        camPoses.add(cams[camNum].camPose);
        camTimestamps.add(cams[camNum].camTimestamp);

    }

    public List<Pose2d> getCameraPoses() {
        return camPoses;
    }

    public List<Double> getCameraTimestamps() {
        return camTimestamps;
    }

}
