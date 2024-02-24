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
        updatePose();
        SmartDashboard.putNumber("Vision X", getVisionPose().getX());
        SmartDashboard.putNumber("Vision Y", getVisionPose().getY());
        SmartDashboard.putBoolean("Sees April Tag", seesAprilTags());
        SmartDashboard.putNumber("Note X", getNoteX());
        SmartDashboard.putNumber("Note Y", getNoteY());
        SmartDashboard.putNumber("Note Area", getNoteArea());
        SmartDashboard.putBoolean("Sees Note", seesNote());
        SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance(Alliance.Blue));
        try {
            SmartDashboard.putNumber("Speaker Angle", getSpeakerAngle(Alliance.Blue).getDegrees());
        } catch (Exception e) {  
        } 
    }

    @Override
    public void stop() {
    }

    public void updatePose() {
        Results results = LimelightHelpers.getLatestResults(VisionConstants.APRILTAG_CAM_NAME).targetingResults;
        
        Pose2d robotPose = results.getBotPose2d_wpiBlue();
        double tl = results.latency_pipeline;
        double cl = results.latency_capture;
        
        if (robotPose.getX() != 0.0 && seesAprilTags()) {
            visionPose = robotPose;
            drive.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.XY_STDS, VisionConstants.XY_STDS, Units.degreesToRadians(VisionConstants.DEG_STDS)));
            drive.getPoseEstimator().addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0));
        }
    }

    public Boolean seesAprilTags(){
        return LimelightHelpers.getTV(VisionConstants.APRILTAG_CAM_NAME);
    }
    
    public Pose2d getVisionPose(){
        return visionPose;
    }

    public double getSpeakerDistance(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        if (!seesAprilTags()){
            return 0;
        }
        return visionPose.getTranslation().getDistance(speakerPose.getTranslation());
    }

    public Rotation2d getSpeakerAngle(Alliance alliance) {
        Pose2d speakerPose = alliance == Alliance.Blue ? VisionConstants.BLUE_SPEAKER_POSE : VisionConstants.RED_SPEAKER_POSE;
        if (!seesAprilTags()){
            return null;
        }
        return Rotation2d.fromRadians(Math.atan2(visionPose.getY() - speakerPose.getY(), visionPose.getX() - speakerPose.getX()));
    }

    public Boolean seesNote(){
        return LimelightHelpers.getTV(VisionConstants.NOTE_CAM_NAME);
    }

    public double getNoteX(){
        return LimelightHelpers.getTX(VisionConstants.NOTE_CAM_NAME);
    }

    public double getNoteY(){
        return LimelightHelpers.getTY(VisionConstants.NOTE_CAM_NAME);
    }

    public double getNoteArea(){
        return LimelightHelpers.getTA(VisionConstants.NOTE_CAM_NAME);
    }

    // public double updateNoteAngle(){
    //     return LimelightHelpers.getTX(VisionConstants.NOTE_CAM_NAME);
    // }
}