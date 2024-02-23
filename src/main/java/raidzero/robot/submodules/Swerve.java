package raidzero.robot.submodules;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import com.choreo.lib.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;

public class Swerve extends Submodule {
    private enum ControlState {
        OPEN_LOOP, PATHING, AUTO_AIM
    };

    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {}

    private static final Vision mVision = Vision.getInstance();

    private SwerveModule mTopRightModule = new SwerveModule();
    private SwerveModule mTopLeftModule = new SwerveModule();
    private SwerveModule mRearLeftModule = new SwerveModule();
    private SwerveModule mRearRightModule = new SwerveModule();

    private Pigeon2 mPigeon = new Pigeon2(SwerveConstants.kImuID);

    private SwerveDrivePoseEstimator mOdometry;

    private PathPlannerTrajectory mCurrentTrajectory; 
    private PPHolonomicDriveController mHolonomicController; 
    private PPHolonomicDriveController testController;
    private ChassisSpeeds mDesiredPathingSpeeds; 
    private Timer mTimer = new Timer();
    private Pose2d mCurrentPose;
    private Pose2d mCurrentAutoPose;

    private ControlState mControlState = ControlState.OPEN_LOOP;

    private final Limelight mLimelight = Limelight.getInstance();
    private ProfiledPIDController mAimAssistYController = new ProfiledPIDController(
        SwerveConstants.kAimAssistController_kP, 
        SwerveConstants.kAimAssistController_kI, 
        SwerveConstants.kAimAssistController_kD, 
        SwerveConstants.kAimAssistControllerConstraints

    );
    // private ProfiledPIDController mSnapController = new ProfiledPIDController(
    //     SwerveConstants.kSnapController_kP, 
    //     SwerveConstants.kSnapController_kI, 
    //     SwerveConstants.kSnapController_kD, 
    //     SwerveConstants.kSnapControllerConstraints
    // );

    private PIDController mSnapController = new PIDController(
        SwerveConstants.kSnapController_kP, 
        SwerveConstants.kSnapController_kI, 
        SwerveConstants.kSnapController_kD
    );

    private Field2d fieldPose = new Field2d();

    private boolean firstPath = true;    

    public void first(){
        firstPath = true;
    }

    public void onStart(double timestamp) {
        mControlState = ControlState.OPEN_LOOP;
        // alliance = DriverStation.getAlliance();
        zero();
        firstPath = true;

        // TEMPORARY
        // setPose(new Pose2d(4,1.2,Rotation2d.fromDegrees(180)));
    }

    public void onInit() {
        Shuffleboard.getTab(Tab.MAIN).add("Pigey", mPigeon).withSize(2, 2).withPosition(4, 4);

        mTopLeftModule.onInit(
            SwerveConstants.kFrontLeftThrottleID,
            SwerveConstants.kFrontLeftAzimuthID,
            SwerveConstants.kFrontLeftEncoderID,
            SwerveConstants.kFrontLeftAzimuthOffset
        );
        mTopRightModule.onInit(
            SwerveConstants.kFrontRightThrottleID,
            SwerveConstants.kFrontRightAzimuthID,
            SwerveConstants.kFrontRightEncoderID,
            SwerveConstants.kFrontRightAzimuthOffset
        );
        mRearLeftModule.onInit(
            SwerveConstants.kRearLeftThrottleID,
            SwerveConstants.kRearLeftAzimuthID,
            SwerveConstants.kRearLeftEncoderID,
            SwerveConstants.kRearLeftAzimuthOffset
        );
        mRearRightModule.onInit(
            SwerveConstants.kRearRightThrottleID,
            SwerveConstants.kRearRightAzimuthID,
            SwerveConstants.kRearRightEncoderID,
            SwerveConstants.kRearRightAzimuthOffset
        );

        mOdometry = new SwerveDrivePoseEstimator(
            SwerveConstants.kKinematics,
            Rotation2d.fromDegrees(mPigeon.getAngle()),
            getModulePositions(),
            DriveConstants.STARTING_POSE,
            DriveConstants.STATE_STDEVS_MATRIX,
            DriveConstants.VISION_STDEVS_MATRIX
        );

        mHolonomicController = new PPHolonomicDriveController(
            new PIDConstants(SwerveConstants.kTranslationController_kP, SwerveConstants.kTranslationController_kD), 
            new PIDConstants(SwerveConstants.kThetaController_kP), 
            /*SwerveConstants.kTestingMaxVelMPS*/ SwerveConstants.kMaxVelMPS,
            0.4
        );
        testController = new PPHolonomicDriveController(
            new PIDConstants(SwerveConstants.kTranslationController_kP), 
            new PIDConstants(SwerveConstants.kThetaController_kP), 
            3.0,
            0.4
        ); 

        mDesiredPathingSpeeds = new ChassisSpeeds();

        mSnapController.enableContinuousInput(-180, 180);
        mSnapController.setTolerance(SwerveConstants.kSnapControllerToleranceDegrees);

        zero();
    }

    @Override
    public void update(double timestamp) {
        if(mControlState == ControlState.PATHING) {
            updatePathing();
        } else if(mControlState == ControlState.AUTO_AIM) {
            
        }

        mTopRightModule.update(timestamp);
        mTopLeftModule.update(timestamp);
        mRearLeftModule.update(timestamp);
        mRearRightModule.update(timestamp);

        mCurrentPose = updateOdometry(timestamp);
        mCurrentAutoPose = mCurrentPose;
        //if (DriverStation.getAlliance().get() == Alliance.Red){
        //    mCurrentAutoPose = new Pose2d(-mCurrentPose.getX(),mCurrentPose.getY(),Rotation2d.fromDegrees(-mCurrentPose.getRotation().getDegrees()));
        //}

        SmartDashboard.putNumber("X pose", mCurrentAutoPose.getX());
        SmartDashboard.putNumber("Y pose", mCurrentAutoPose.getY());
        SmartDashboard.putNumber("Theta pose", mCurrentAutoPose.getRotation().getDegrees());

        
        fieldPose.setRobotPose(mCurrentPose);

        // This needs to be moved somewhere else.....
        SmartDashboard.putData(fieldPose);

        //SmartDashboard.putNumber("X pose", mOdometry.getEstimatedPosition().getX());
        //SmartDashboard.putNumber("Y pose", mOdometry.getEstimatedPosition().getY());
        //SmartDashboard.putNumber("Theta pose", mOdometry.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("Raw Pigeon Rotation2d", mPigeon.getRotation2d().getDegrees());

        // if(vision.getRobotPose() != null) {
        // setPose(vision.getRobotPose());
        // }
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    @Override
    public void run() {
        mTopRightModule.run();
        mTopLeftModule.run();
        mRearLeftModule.run();
        mRearRightModule.run();
    }

    @Override
    public void stop() {
        mControlState = ControlState.OPEN_LOOP;
        mTopRightModule.stop();
        mTopLeftModule.stop();
        mRearLeftModule.stop();
        mRearRightModule.stop();
    }

    /**
     * Resets the sensor(s) to zero.
     */
    @Override
    public void zero() {
        if (DriverStation.getAlliance().get() == Alliance.Blue)
            zeroHeading(0);
        else if (DriverStation.getAlliance().get() == Alliance.Red)
            //zeroHeading(180);
            zeroHeading(0);



        // setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(pigeon.getAngle()))));

        // mPigeon.setYaw(0.0);

        mTopRightModule.zero();
        mTopLeftModule.zero();
        mRearLeftModule.zero();
        mRearRightModule.zero();

        mOdometry.resetPosition(mPigeon.getRotation2d(), getModulePositions(), new Pose2d());

        // mSnapController.reset(mPigeon.getRotation2d().getDegrees());
    }

    /**
     * Zeroes the heading of the swerve at set Yaw
     */
    public void zeroHeading(double q) {
        mPigeon.setYaw(q, Constants.kCANTimeoutMs);
    }

    // public void zeroTele(double q) {
    //     mPigeon.setYaw(q, Constants.TIMEOUT_MS);
    //     setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(mPigeon.getAngle()))));
    // }

    public double getYawRate() {
        return mPigeon.getRate();
    }

    public Field2d getField() {
        return fieldPose;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            mTopLeftModule.getPosition(),
            mTopRightModule.getPosition(),
            mRearLeftModule.getPosition(),
            mRearRightModule.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            mTopLeftModule.getState(),
            mTopRightModule.getState(),
            mRearLeftModule.getState(),
            mRearRightModule.getState()
        };
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[] {
            mTopLeftModule, 
            mTopRightModule, 
            mRearLeftModule, 
            mRearRightModule
        };
    }

    public static double deadband(double input) {
        if (Math.abs(input) < 0.7) {
            return 0.0;
        }
        return input;
    }

    public void setPose(Pose2d pose) {
        mOdometry.resetPosition(Rotation2d.fromDegrees(mPigeon.getAngle()), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return mOdometry.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return mOdometry;
    }

    public synchronized void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // visionMeasurementStdDevs = new MatBuilder<N3, N1>(Nat.N3(),
            // Nat.N1()).fill(0.2, 0.2, 0.1);
            // mOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
            mOdometry.addVisionMeasurement(
                    visionRobotPoseMeters,
                    timestampSeconds,
                    visionMeasurementStdDevs);
        } catch (Exception e) {
            System.out.println("Cholesky decomposition failed, reverting...:");
        }
    }

    /**
     * Updates odometry
     * 
     * @param timestamp
     * 
     * @return current position
     */
    private Pose2d updateOdometry(double timestamp) {
        try {
            SwerveModulePosition[] reversedPositions = new SwerveModulePosition[] {
                new SwerveModulePosition(-getModulePositions()[0].distanceMeters, getModulePositions()[0].angle), 
                new SwerveModulePosition(-getModulePositions()[1].distanceMeters, getModulePositions()[1].angle), 
                new SwerveModulePosition(-getModulePositions()[2].distanceMeters, getModulePositions()[2].angle), 
                new SwerveModulePosition(-getModulePositions()[3].distanceMeters, getModulePositions()[3].angle), 
            };
            return mOdometry.updateWithTime(timestamp, mPigeon.getRotation2d(), reversedPositions/*getModulePositions()*/);
            // return mOdometry.updateWithTime(timestamp,
            //         Rotation2d.fromDegrees(mPigeon.getAngle()),
            //         reversedPositions);
            // return mOdometry.updateWithTime(timestamp,
            //         Rotation2d.fromDegrees(mPigeon.getAngle()),
            //         getModulePositions());
        } catch (Exception e) {
            System.out.println(e);
            return mOdometry.getEstimatedPosition();
        }
    }

    // IMPORTANT
    /**
     * Drives robot (primarily used for teleop manual control)
     * 
     * @param xSpeed        speed in x direction
     * @param ySpeed        speed in y direction
     * @param angularSpeed  turn speed
     * @param fieldOriented
     */
    public void teleopDrive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented, Rotation2d snapAngle, boolean autoAim, boolean aimAssist) {
        if (mControlState == ControlState.AUTO_AIM) {
            return;
        }
        mControlState = ControlState.OPEN_LOOP;

        if(snapAngle != null) {
            angularSpeed = -mSnapController.calculate(mCurrentPose.getRotation().getDegrees(), snapAngle.getDegrees());
            SmartDashboard.putNumber("Snap Controller Output Speed", angularSpeed);
        } 
        // else {
        //     mSnapController.reset(mPigeon.getRotation2d().getDegrees());
        // }

        if(autoAim && mVision.getSpeakerAngle(Alliance.Blue) != null) {
            // angularSpeed = mSnapController.calculate(-mPigeon.getRotation2d().getDegrees(), -mVision.getSpeakerAngle(Alliance.Blue).getDegrees());
            angularSpeed = (-mVision.getSpeakerAngle(Alliance.Blue).getDegrees() + mPigeon.getRotation2d().getDegrees()) * 0.15;
        }

        // setOpenLoopSpeeds(new ChassisSpeeds(xSpeed, ySpeed, angularSpeed), fieldOriented);
        if(aimAssist && mLimelight.hasTarget()) {
            double y = mAimAssistYController.calculate(mLimelight.getTx(), 0.0);
            ChassisSpeeds driverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0.0, angularSpeed, mPigeon.getRotation2d());
            ChassisSpeeds aimAssistSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0.0, y, 0.0, mPigeon.getRotation2d());
            setClosedLoopSpeeds(driverSpeeds.plus(aimAssistSpeeds), false);
            // setClosedLoopSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(0.0, y, 0.0, mPigeon.getRotation2d()), false);
        } else {
            setClosedLoopSpeeds(new ChassisSpeeds(xSpeed, ySpeed, angularSpeed), fieldOriented);
            // setOpenLoopSpeeds(new ChassisSpeeds(xSpeed, ySpeed, angularSpeed), fieldOriented);
        }
    }

    /**
     * Follow path
     * 
     * @param trajectory desired path
     */
    public void followPath(PathPlannerTrajectory trajectory) {
        if (mControlState == ControlState.PATHING) {
            // return;
        }
        if (firstPath) {
            setPose(trajectory.getInitialTargetHolonomicPose());
            firstPath = false;
        }
        mControlState = ControlState.PATHING;
        mCurrentTrajectory = trajectory;

        mTimer.reset();
        mTimer.start();
    }

    private void updatePathing() {
        PathPlannerTrajectory.State state = (PathPlannerTrajectory.State) mCurrentTrajectory.sample(mTimer.get());

        /*if(DriverStation.getAlliance().get() == Alliance.Red){
            state.headingAngularVelocityRps*=-1;
            state.positionMeters = new Translation2d(-state.positionMeters.getX(),state.positionMeters.getY());
            state.heading = new Rotation2d(-state.heading.getRadians());
            state.targetHolonomicRotation = new Rotation2d(-state.targetHolonomicRotation.getRadians());
        }
        */
        //SmartDashboard.putNumber("state vel", state.velocityMps);
        mHolonomicController.setEnabled(true); //false, doesnt turn when only ff
        testController.setEnabled(true);
        // PathPlannerPath.fromChoreoTrajectory()
        ChassisSpeeds desiredSpeeds = mHolonomicController.calculateRobotRelativeSpeeds(/*mCurrentAutoPose*/ mCurrentPose, state);
        SmartDashboard.putNumber("Desired State X", state.getTargetHolonomicPose().getX());
        SmartDashboard.putNumber("error", mHolonomicController.getPositionalError());
        //ChassisSpeeds trash = testController.calculateRobotRelativeSpeeds(mCurrentPose, state);
        // desiredSpeeds.times(-1.0);
        
        //if(DriverStation.getAlliance().get() == Alliance.Red){
        //    desiredSpeeds.vxMetersPerSecond = -desiredSpeeds.vxMetersPerSecond;
        //    desiredSpeeds.omegaRadiansPerSecond = -desiredSpeeds.omegaRadiansPerSecond;
        //}

        mDesiredPathingSpeeds = desiredSpeeds;
        setClosedLoopSpeeds(mDesiredPathingSpeeds,false); //true

//// setOpenLoopSpeeds(desiredSpeeds);
//SmartDashboard.putNumber("desired heading", state.targetHolonomicRotation.getDegrees());
//if(state.holonomicAngularVelocityRps.isPresent())SmartDashboard.putNumber("desired holonomicAngularVelocityRps", state.holonomicAngularVelocityRps.get());
//SmartDashboard.putNumber("desired omegaRadiansPerSecond", desiredSpeeds.omegaRadiansPerSecond);
SmartDashboard.putNumber("desired xmps", desiredSpeeds.vxMetersPerSecond);
SmartDashboard.putNumber("desired ymps", desiredSpeeds.vyMetersPerSecond);
////SmartDashboard.putNumber("ff xmps", trash.vxMetersPerSecond);
////SmartDashboard.putNumber("ff ymps", trash.vyMetersPerSecond);
//SmartDashboard.putNumber("trottle dist from ideal", aConstants.SwerveConstants.kMetersToThrottleRot*4-mTopRightModule.throttlePosTravelled());
    }



    /**
     * Get total path time
     * 
     * @return path time
     */
    public double getPathingTime() {
        return mTimer.get();
    }

    /**
     * Check if robot has finished pathing
     * 
     * @return robot pathing state
     */
    public boolean isFinishedPathing() {
        // if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
        //     if (mTimer.hasElapsed(currentTrajectory.getTotalTimeSeconds())) {
        //         return true;
        //     }
        // }
        // return false;

         if(/*mHolonomicController.getPositionalError() < 0.05 && */mTimer.hasElapsed(mCurrentTrajectory.getTotalTimeSeconds()+0.2)) {
             System.out.println("Done Pathing!");
             return true;
         }
        return false;
    }

    // IMPORTANT

    public void setOpenLoopSpeeds(ChassisSpeeds speeds, boolean fieldOriented) {
        if(fieldOriented) {
            // IMPORTANT - pigeon might need * -1
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond,
                mPigeon.getRotation2d()
                // DriverStation.getAlliance().get() == Alliance.Blue ? mPigeon.getRotation2d() : mPigeon.getRotation2d().minus(Rotation2d.fromDegrees(180))
            );
        } 

        // test
        ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        mTopLeftModule.setOpenLoopState(desiredState[0]);
        mTopRightModule.setOpenLoopState(desiredState[1]);
        mRearLeftModule.setOpenLoopState(desiredState[2]);
        mRearRightModule.setOpenLoopState(desiredState[3]);
    }

    // check
    public void setClosedLoopSpeeds(ChassisSpeeds speeds, boolean fieldOriented) {
        if(fieldOriented) {
            // IMPORTANT - pigeon might need * -1
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond,
                mPigeon.getRotation2d()
                // DriverStation.getAlliance().get() == Alliance.Blue ? mPigeon.getRotation2d() : mPigeon.getRotation2d().minus(Rotation2d.fromDegrees(180))
            );
        } 

        ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.kRealisticMaxVelMPS);
        mTopLeftModule.setClosedLoopState(desiredState[0]);
        mTopRightModule.setClosedLoopState(desiredState[1]);
        mRearLeftModule.setClosedLoopState(desiredState[2]);
        mRearRightModule.setClosedLoopState(desiredState[3]);
    }

    // public void setAutoAimLocation(AutoAimLocation location) {
    //     autoAimController.setTarget(getPose(), location, true);
    // }

    // public void setClosedLoopSpeeds(ChassisSpeeds speeds, boolean fieldOriented) {
    //     if(fieldOriented) {
    //         // IMPORTANT - pigeon might need * -1
    //         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //             speeds.vxMetersPerSecond, 
    //             speeds.vyMetersPerSecond, 
    //             speeds.omegaRadiansPerSecond,
    //             mPigeon.getRotation2d()
    //         );
    //     } 

    //     ChassisSpeeds.discretize(speeds, 0.02);

    //     SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.kRealisticMaxVelMPS);
    //     mTopLeftModule.setClosedLoopState(desiredState[0]);
    //     mTopRightModule.setClosedLoopState(desiredState[1]);
    //     mRearLeftModule.setClosedLoopState(desiredState[2]);
    //     mRearRightModule.setClosedLoopState(desiredState[3]);
    // }

    public ChassisSpeeds getOpenLoopSpeeds() {
        // return SwerveConstants.KINEMATICS.toChassisSpeeds(
        //         new SwerveModuleState(mTopLeftModule.getThrottlePercentSpeed(),
        //                 Rotation2d.fromDegrees(mTopLeftModule.getRotorAngle())),
        //         new SwerveModuleState(mTopRightModule.getThrottlePercentSpeed(),
        //                 Rotation2d.fromDegrees(mTopRightModule.getRotorAngle())),
        //         new SwerveModuleState(mRearLeftModule.getThrottlePercentSpeed(),
        //                 Rotation2d.fromDegrees(mRearLeftModule.getRotorAngle())),
        //         new SwerveModuleState(mRearRightModule.getThrottlePercentSpeed(),
        //                 Rotation2d.fromDegrees(mRearRightModule.getRotorAngle())));
        return null;
    }


    // public void setAutoAimLocation(AutoAimLocation location) {
    //     autoAimController.setTarget(getPose(), location, true);
    // }
    // public void enableAutoAimController(boolean isEnabled) {
    //     if (isEnabled) {
    //         mControlState = ControlState.AUTO_AIM;
    //         autoAimController.enable(isEnabled);
    //     } else {
    //         mControlState = ControlState.OPEN_LOOP;
    //     }
    // }

    // public void setFortniteAutoAimTM(double yDist, double xSpeed) {
    //     enableAutoAimController(true);
    //     Rotation2d desiredAngle = new Rotation2d();
    //     // if (DriverStation.getAlliance() == Alliance.Blue) {
    //     //     desiredAngle = Rotation2d.fromDegrees(180);
    //     // } else {
    //     //     desiredAngle = Rotation2d.fromDegrees(0);
    //     // }
    //     autoAimController.setTarget(yDist, desiredAngle, xSpeed);
    // }

    /**
     * Test swerve modules
     * 
     * @param quadrant       module quadrant [I, II, III, IV]
     * @param throttleOutput output of throttle motor
     * @param rotorOutput    output of rotor motor
     */
    public void testModule(int quadrant, double throttleOutput, double rotorOutput) {
        if (quadrant == 1) {
            mTopLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 2) {
            mRearLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 3) {
            mRearRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else {
            mTopRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        }
    }
}