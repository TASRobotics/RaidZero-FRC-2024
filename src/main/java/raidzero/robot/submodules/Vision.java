package raidzero.robot.submodules;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.utils.VisionCamera;

public class Vision extends Submodule {
    private static Vision instance;
    private static final Swerve drive = Swerve.getInstance();

    private VisionCamera camera1;
    private VisionCamera camera2;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        camera1 = new VisionCamera(VisionConstants.CAM1_NAME, VisionConstants.ROBOT_TO_CAM1);
        camera2 = new VisionCamera(VisionConstants.CAM2_NAME, VisionConstants.ROBOT_TO_CAM2);
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
        camera1.updatePose();
        camera2.updatePose();
    }

    @Override
    public void stop() {
    }
}
