package raidzero.robot.submodules;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.wrappers.LimelightHelpers;
import raidzero.robot.wrappers.LimelightHelpers.LimelightResults;
import raidzero.robot.wrappers.LimelightHelpers.Results;

public class Vision extends Submodule {
    private static Vision instance;
    private static final Swerve drive = Swerve.getInstance();

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
        updateVisionMeasurement();
    }

    @Override
    public void stop() {
    }

    public void updateVisionMeasurement(){
        Results results = LimelightHelpers.getLatestResults(VisionConstants.NAME).targetingResults;
        
        Pose2d robotPose = results.getBotPose2d();
        double tl = results.latency_pipeline;
        double cl = results.latency_capture;
        
        if (robotPose.getX() == 0.0 && hasTarget()) {
            drive.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.XY_STDS, VisionConstants.XY_STDS, Units.degreesToRadians(VisionConstants.DEG_STDS)));
            drive.getPoseEstimator().addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
        }
    }

    public Boolean hasTarget(){
        return LimelightHelpers.getTV(VisionConstants.NAME);
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER : VisionConstants.RED_SPEAKER;
        if (!hasTarget()){
            return 0;
        }
        return drive.getPose().getTranslation().getDistance(speakerPose.getTranslation());
    }
}
