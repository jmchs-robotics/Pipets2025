package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotToCamTransforms;

public class VisionSubsystem extends SubsystemBase {

    private BulldogCamera[] cams = {
        new BulldogCamera("BulldogCam1", RobotToCamTransforms.kCam1Transform),
        // new BulldogCamera("BulldogCam2", RobotToCamTransforms.kCam2Transform)
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

        SmartDashboard.putNumber("Cam To Tag X", cams[0].camToTagX);
        SmartDashboard.putNumber("Cam To Tag Y", cams[0].camToTagY);
        SmartDashboard.putNumber("Cam To Tag Yaw", cams[0].camToTagYaw);

    }

    public void processCamera(int camNum) {

        camPoses.add(cams[camNum].camPose);
        camTimestamps.add(cams[camNum].camTimestamp);

    }

    public BulldogCamera getCamera(int camNum) {
        return cams[camNum];
    }

    public void setReferencePose(Pose2d pose) {
        for (BulldogCamera cam : cams) {
            cam.setReferencePose(pose);
        }
    }

    public List<Pose2d> getCameraPoses() {
        return camPoses;
    }

    public List<Double> getCameraTimestamps() {
        return camTimestamps;
    }

    public double getMinDistance(int num) {
        return cams[num].getMinDistance();
    }

}
