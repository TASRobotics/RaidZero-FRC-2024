package raidzero.robot.submodules;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.wrappers.LimelightHelpers;
import raidzero.robot.wrappers.LimelightHelpers.Results;

public class Vision extends Submodule {
    private static Vision instance;
    private static final Swerve drive = Swerve.getInstance();

    private Pose2d visionPose = new Pose2d();

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
        SmartDashboard.putNumber("Vision X", visionPose.getX());
        SmartDashboard.putNumber("Vision Y", visionPose.getY());
        SmartDashboard.putBoolean("Vision Target", hasTarget());
        SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance(Alliance.Blue));
        try {
            SmartDashboard.putNumber("Speaker Angle", getSpeakerAngle(Alliance.Blue).getDegrees());
        } catch (Exception e) {  
        } 
        updatePose();
    }

    @Override
    public void stop() {
    }

    public void updatePose() {
        Results results = LimelightHelpers.getLatestResults(VisionConstants.NAME).targetingResults;

        Pose2d robotPose = results.getBotPose2d_wpiBlue();
        if (hasTarget()) {
            visionPose = robotPose;
        }
    }

    public void updateVisionMeasurement(){
        Results results = LimelightHelpers.getLatestResults(VisionConstants.NAME).targetingResults;
        
        Pose2d robotPose = results.getBotPose2d_wpiBlue();
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

    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER : VisionConstants.RED_SPEAKER;
        if (!hasTarget()){
            return 0;
        }
        return visionPose.getTranslation().getDistance(speakerPose.getTranslation());
    }

    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER : VisionConstants.RED_SPEAKER;
        if (!hasTarget()){
            return null;
        }
        return Rotation2d.fromRadians(Math.atan2(visionPose.getY() - speakerPose.getY(), visionPose.getX() - speakerPose.getX()));
    }
}
