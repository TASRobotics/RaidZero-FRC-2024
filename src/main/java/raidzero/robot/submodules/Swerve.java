package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.utils.AutoAimController;
import raidzero.robot.utils.AutoAimController.AutoAimLocation;

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

    // private static final Vision vision = Vision.getInstance();

    private SwerveModule mTopRightModule = new SwerveModule();
    private SwerveModule mTopLeftModule = new SwerveModule();
    private SwerveModule mRearLeftModule = new SwerveModule();
    private SwerveModule mRearRightModule = new SwerveModule();

    private Pigeon2 mPigeon = new Pigeon2(SwerveConstants.kImuID, Constants.kCANBusName);

    private SwerveDrivePoseEstimator mOdometry;

    private PathPlannerTrajectory mCurrentTrajectory; 
    private PPHolonomicDriveController mHolonomicController; 
    private PPHolonomicDriveController testController;
    private ChassisSpeeds mDesiredPathingSpeeds; 
    private Timer mTimer = new Timer();
    private Pose2d mCurrentPose;

    private ControlState mControlState = ControlState.OPEN_LOOP;







    
    private Pose2d prevPose;
    private Field2d fieldPose = new Field2d();

    private PathPlannerTrajectory currentTrajectory;
    private boolean firstPath = true;
    private boolean overLimit = false;
    // private Rotation2d targetAngle;
    private PIDController xController, yController, thetaController;
    private Alliance alliance;
    private double prevX;

    private Pose2d desiredAutoAimPose;
    private PIDController autoAimXController, autoAimYController;
    private ProfiledPIDController autoAimThetaController, snapController;
    private TrajectoryConfig autoAimTrajectoryConfig;
    private AutoAimController autoAimController;

    

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
            new PIDConstants(SwerveConstants.kTranslationController_kP), 
            new PIDConstants(SwerveConstants.kThetaController_kP), 
            3.0,
            0.4
        );
        testController = new PPHolonomicDriveController(
            new PIDConstants(SwerveConstants.kTranslationController_kP), 
            new PIDConstants(SwerveConstants.kThetaController_kP), 
            3.0,
            0.4
        ); 

        mDesiredPathingSpeeds = new ChassisSpeeds();

        // snapController = new ProfiledPIDController(1.25, 0, 0.15, new TrapezoidProfile.Constraints(
        //         SwerveConstants.MAX_ANGULAR_VEL_RPS, SwerveConstants.MAX_ANGULAR_ACCEL_RPSPS * 2));
        // snapController.enableContinuousInput(-Math.PI, Math.PI);
        // snapController.reset(getPose().getRotation().getRadians(), 0);

        // xController = new PIDController(SwerveConstants.kXControllerP, 0, 0);
        // yController = new PIDController(SwerveConstants.YCONTROLLER_KP, 0, 0);
        // thetaController = new PIDController(SwerveConstants.THETACONTROLLER_KP, 0, SwerveConstants.THETACONTROLLER_KD);
        // xController.setTolerance(SwerveConstants.XCONTROLLER_TOLERANCE);
        // yController.setTolerance(SwerveConstants.YCONTROLLER_TOLERANCE);
        // thetaController.setTolerance(SwerveConstants.THETACONTROLLER_TOLERANCE);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // autoAimXController = new PIDController(SwerveConstants.AA_XCONTROLLER_KP, 0.0, 0.0);
        // autoAimYController = new PIDController(SwerveConstants.AA_YCONTROLLER_KP, 0.0, 0.0);
        // autoAimThetaController = new ProfiledPIDController(SwerveConstants.AA_THETACONTROLLER_KP, 0.0, 0.0,
        //         new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_VEL_RPS,
        //                 SwerveConstants.MAX_ANGULAR_ACCEL_RPSPS));
        // autoAimTrajectoryConfig = new TrajectoryConfig(SwerveConstants.MAX_DRIVE_VEL_MPS,
        //         SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
        // autoAimController = new AutoAimController(autoAimXController, autoAimYController, autoAimThetaController,
        //         autoAimTrajectoryConfig);
        // autoAimController.setTolerance(new Pose2d(
        //         SwerveConstants.AA_XCONTROLLER_TOLERANCE,
        //         SwerveConstants.AA_YCONTROLLER_TOLERANCE,
        //         Rotation2d.fromRadians(SwerveConstants.AA_THETACONTROLLER_TOLERANCE)));

        zero();

        // Auto Balance Constants
        prevPose = new Pose2d();
        prevX = 0;

        // PathPlannerServer.startServer(5811);
    }

    String control_state = "nada";

    @Override
    public void update(double timestamp) {
        if(mControlState == ControlState.PATHING) {
            updatePathing();
        }

        mTopRightModule.update(timestamp);
        mTopLeftModule.update(timestamp);
        mRearLeftModule.update(timestamp);
        mRearRightModule.update(timestamp);

        prevPose = mCurrentPose;

        mCurrentPose = updateOdometry(timestamp);
        fieldPose.setRobotPose(mCurrentPose);

        // This needs to be moved somewhere else.....
        SmartDashboard.putData(fieldPose);

        SmartDashboard.putNumber("X pose", mOdometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y pose", mOdometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Theta pose", mOdometry.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("0 Swerve Module Vel", -mTopLeftModule.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("0 Desired Velocity", mDesiredPathingSpeeds.vxMetersPerSecond);


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
        // if (alliance == Alliance.Blue)
        //     zeroHeading(180);
        // else if (alliance == Alliance.Red)
        //     zeroHeading(0);
        // setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(pigeon.getAngle()))));

        mPigeon.setYaw(0.0);

        mTopRightModule.zero();
        mTopLeftModule.zero();
        mRearLeftModule.zero();
        mRearRightModule.zero();

        mOdometry.resetPosition(mPigeon.getRotation2d(), getModulePositions(), new Pose2d());
    }

    /**
     * Zeroes the heading of the swerve at set Yaw
     */
    public void zeroHeading(double q) {
        mPigeon.setYaw(q, Constants.TIMEOUT_MS);
    }

    public void zeroTele(double q) {
        mPigeon.setYaw(q, Constants.TIMEOUT_MS);
        setPose(new Pose2d(new Translation2d(1.76, 1.477), new Rotation2d(Math.toRadians(mPigeon.getAngle()))));
    }

    public double getYawRate() {
        return mPigeon.getRate();
    }

    // public void lockTo90() {
    // if (Math.abs(pigeon.getYaw()-180) > 1 || Math.abs(pigeon.getYaw()-0) > 1)
    // drive(0,0,0.3, false);
    // // pigeon.setYaw(q, Constants.TIMEOUT_MS);
    // // setPose(new Pose2d(new Translation2d(1.76,1.477), new
    // Rotation2d(Math.toRadians(pigeon.getAngle()))));
    // }

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

    public Pose2d getPrevPose() {
        return prevPose;
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
            // SwerveModulePosition[] reversedPositions = new SwerveModulePosition[] {
            //     new SwerveModulePosition(-getModulePositions()[0].distanceMeters, getModulePositions()[0].angle), 
            //     new SwerveModulePosition(-getModulePositions()[1].distanceMeters, getModulePositions()[1].angle), 
            //     new SwerveModulePosition(-getModulePositions()[2].distanceMeters, getModulePositions()[2].angle), 
            //     new SwerveModulePosition(-getModulePositions()[3].distanceMeters, getModulePositions()[3].angle)
            // };
            return mOdometry.updateWithTime(timestamp,
                    Rotation2d.fromDegrees(mPigeon.getAngle()),
                    getModulePositions());
        } catch (Exception e) {
            System.out.println(e);
            return mOdometry.getEstimatedPosition();
        }
    }

    /**
     * Drives robot (primarily used for teleop manual control)
     * 
     * @param xSpeed        speed in x direction
     * @param ySpeed        speed in y direction
     * @param angularSpeed  turn speed
     * @param fieldOriented
     */
    public void teleopDrive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented) {
        if (mControlState == ControlState.AUTO_AIM) {
            return;
        }
        mControlState = ControlState.OPEN_LOOP;
        var targetState = SwerveConstants.kKinematics.toSwerveModuleStates(
                fieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, angularSpeed,
                                Rotation2d.fromDegrees(-mPigeon.getAngle()))
                        : new ChassisSpeeds(xSpeed, ySpeed, angularSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(targetState, 1);

        mTopLeftModule.setOpenLoopState(targetState[0]);
        mTopRightModule.setOpenLoopState(targetState[1]);
        mRearLeftModule.setOpenLoopState(targetState[2]);
        mRearRightModule.setOpenLoopState(targetState[3]);
    }

    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented, Rotation2d snapAngle) {
        if (Math.abs(angularSpeed) > 0.1) {
            teleopDrive(xSpeed, ySpeed, angularSpeed, fieldOriented);
        } else {
            double thetaOutput = snapController.calculate(getPose().getRotation().getRadians(), snapAngle.getRadians());
            teleopDrive(xSpeed, ySpeed, thetaOutput, fieldOriented);
        }
    }

    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented, boolean snap) {
        if (snap && Math.abs(angularSpeed) < 0.1) {
            double thetaOutput = snapController.calculate(getPose().getRotation().getRadians(),
                    /*DriverStation.getAlliance() == Alliance.Blue ? Math.PI :*/ 0);
            teleopDrive(xSpeed, ySpeed, thetaOutput, fieldOriented);
        } else {
            teleopDrive(xSpeed, ySpeed, angularSpeed, fieldOriented);
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

    /**
     * A better updatePathing(), featuring:
     * - actually allows the robot to turn
     * - actually reads turn changes from the pathplanner trajectory
     * - fully feedback, no more weird feedforward stuff that doesnt actually work
     */
    private void updatePathingOld() {
        PathPlannerTrajectory.State state = (PathPlannerTrajectory.State) currentTrajectory.sample(mTimer.get());
        double xSpeed = xController.calculate(getPose().getX(), state.positionMeters.getX());
        double ySpeed = yController.calculate(getPose().getY(), state.positionMeters.getY());
        double thetaSpeed = thetaController.calculate(getPose().getRotation().getRadians(),
                state.targetHolonomicRotation.getRadians());
        // Math
        // SmartDashboard.putNumber("theta speed", thetaSpeed);

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                getPose().getRotation());
        // PathPlannerServer.sendPathFollowingData(state.poseMeters, getPose());

        SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        mTopLeftModule.setOpenLoopState(desiredState[0]);
        mTopRightModule.setOpenLoopState(desiredState[1]);
        mRearLeftModule.setOpenLoopState(desiredState[2]);
        mRearRightModule.setOpenLoopState(desiredState[3]);
    }

    private void updatePathing() {
        PathPlannerTrajectory.State state = (PathPlannerTrajectory.State) mCurrentTrajectory.sample(mTimer.get());
         mHolonomicController.setEnabled(true); //false, doesnt turn when only ff
         testController.setEnabled(false);
        // PathPlannerPath.fromChoreoTrajectory()
        ChassisSpeeds desiredSpeeds = mHolonomicController.calculateRobotRelativeSpeeds(mCurrentPose, state);
        ChassisSpeeds trash = testController.calculateRobotRelativeSpeeds(mCurrentPose, state);
        //desiredSpeeds.times(-1.0);
        mDesiredPathingSpeeds = desiredSpeeds;
        setClosedLoopSpeeds(mDesiredPathingSpeeds,true);
        // setOpenLoopSpeeds(desiredSpeeds);
        SmartDashboard.putNumber("desired heading", state.targetHolonomicRotation.getDegrees());
        if(state.holonomicAngularVelocityRps.isPresent())SmartDashboard.putNumber("desired holonomicAngularVelocityRps", state.holonomicAngularVelocityRps.get());
        SmartDashboard.putNumber("desired omegaRadiansPerSecond", desiredSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("desired xmps", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("desired ymps", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("ff xmps", trash.vxMetersPerSecond);
        SmartDashboard.putNumber("ff ymps", trash.vyMetersPerSecond);
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

        if(/*mHolonomicController.getPositionalError() < 0.05 && */mTimer.hasElapsed(mCurrentTrajectory.getTotalTimeSeconds())) {
            System.out.println("Done Pathing!");
            return true;
        }
        return false;
    }

    public void setOpenLoopSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        mTopLeftModule.setOpenLoopState(desiredState[0]);
        mTopRightModule.setOpenLoopState(desiredState[1]);
        mRearLeftModule.setOpenLoopState(desiredState[2]);
        mRearRightModule.setOpenLoopState(desiredState[3]);
    }

    public void setClosedLoopSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredState = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.kRealisticMaxVelMPS);
        mTopLeftModule.setClosedLoopState(desiredState[0]);
        mTopRightModule.setClosedLoopState(desiredState[1]);
        mRearLeftModule.setClosedLoopState(desiredState[2]);
        mRearRightModule.setClosedLoopState(desiredState[3]);
    }

    public void setClosedLoopSpeeds(ChassisSpeeds speeds, boolean fieldOriented) {
        if(fieldOriented) {
            // IMPORTANT - pigeon might need * -1
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond,
                mPigeon.getRotation2d()
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

    public void setAutoAimLocation(AutoAimLocation location) {
        autoAimController.setTarget(getPose(), location, true);
    }

    public void enableAutoAimController(boolean isEnabled) {
        if (isEnabled) {
            mControlState = ControlState.AUTO_AIM;
            autoAimController.enable(isEnabled);
        } else {
            mControlState = ControlState.OPEN_LOOP;
        }
    }

    public void setFortniteAutoAimTM(double yDist, double xSpeed) {
        enableAutoAimController(true);
        Rotation2d desiredAngle = new Rotation2d();
        // if (DriverStation.getAlliance() == Alliance.Blue) {
        //     desiredAngle = Rotation2d.fromDegrees(180);
        // } else {
        //     desiredAngle = Rotation2d.fromDegrees(0);
        // }
        autoAimController.setTarget(yDist, desiredAngle, xSpeed);
    }

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