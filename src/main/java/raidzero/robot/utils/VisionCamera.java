package raidzero.robot.utils;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Transform3d;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;

public class VisionCamera {
    private static final Swerve drive = Swerve.getInstance();

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private EstimatedRobotPose pose;

    public VisionCamera(String name, Transform3d robotToCam) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.STRATEGY, camera, robotToCam);
    }

    public void updatePose(){
        pose = poseEstimator.update().orElse(null);
        if (pose != null) {
            drive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, VisionConstants.VISION_STD_DEVS);
        }
    }
}
