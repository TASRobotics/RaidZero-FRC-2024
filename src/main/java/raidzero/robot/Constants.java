package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.InterpolatingTreeMap;

public class Constants {
    /**
     * Swerve Constants
     */
    public static final class SwerveConstants {
        /** Device IDs */
        public static final int kFrontLeftThrottleID = 1;
        public static final int kFrontRightThrottleID = 7;
        public static final int kRearLeftThrottleID = 3;
        public static final int kRearRightThrottleID = 5;

        public static final int kFrontLeftAzimuthID = 2;
        public static final int kFrontRightAzimuthID = 8;
        public static final int kRearLeftAzimuthID = 4;
        public static final int kRearRightAzimuthID = 6;

        public static final int kFrontLeftEncoderID = 1;
        public static final int kFrontRightEncoderID = 4;
        public static final int kRearLeftEncoderID = 2;
        public static final int kRearRightEncoderID = 3;

        public static final int kImuID = 0;

        /* Red Arm Offsets */
        public static final double kFrontLeftAzimuthOffset = -0.463379;
        public static final double kFrontRightAzimuthOffset = -0.565674 + 0.5;
        public static final double kRearLeftAzimuthOffset = -0.090088;
        public static final double kRearRightAzimuthOffset = -0.674316 + 0.5;

        /* Alpha Offsets */
        // public static final double kFrontLeftAzimuthOffset = 0.192383;
        // public static final double kFrontRightAzimuthOffset = 0.397949 + 0.5;
        // public static final double kRearLeftAzimuthOffset = 0.255615;
        // public static final double kRearRightAzimuthOffset = -0.205811 + 0.5;

        public static final double kThrottleReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double kAzimuthReduction = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double kWheelDiameterMeters = 0.1016;

        public static final double kMaxVelMPS = 4.959668;
        public static final double kRealisticMaxVelMPS = 4.2;
        public static final double kTestingMaxVelMPS = 3.0;
        public static final double kTestingMaxAccelMPSPS = 3.0;

        // 20.75 OR 22.75 inches
        public static final double kTrackwidthMeters = Units.inchesToMeters(22.75);
        public static final double kWheelbaseMeters = Units.inchesToMeters(22.75);
        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0)
        );

        public static final double kMaxAngularVelRPS = kMaxVelMPS * Math.sqrt(2) * kTrackwidthMeters;


        public static final int kAzimuthPositionPIDSlot = 0;
        public static final double kAzimuth_kP = 40.0;// .75
        public static final double kAzimuth_kI = 0.0;
        public static final double kAzimuth_kD = 0.0;// 5.0
        public static final double kAzimuthPIDUpdateHz = 1000.0;

        public static final int kThrottleVelPIDSlot = 0;
        public static final double kThrottle_kP = 0.5;
        public static final double kThrottle_kI = 0.0;
        public static final double kThrottle_kD = 0.0;
        public static final double kThrottle_kV = 2.4;
        public static final double kThrottle_kA = 1.5;
        public static final double kThrottlePIDUpdateHz = 1000.0;

        public static final double kTranslationController_kP = 5.0;
        public static final double kTranslationController_kD = 0.0;
        public static final double kThetaController_kP = 2.0;
        public static final double kXControllerTolerance = 0.1;
        public static final double kYControllerTolerance = 0.1;
        public static final double kThetaControllerTolerance = Math.toRadians(5);

        public static final double kSnapController_kP = 0.1;
        public static final double kSnapController_kI = 0.0;
        public static final double kSnapController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kSnapControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxAngularVelRPS, kMaxAngularVelRPS);

        public static final double kAimAssistController_kP = 0.1;
        public static final double kAimAssistController_kI = 0.0;
        public static final double kAimAssistController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kAimAssistControllerConstraints = 
            new TrapezoidProfile.Constraints(kRealisticMaxVelMPS, kRealisticMaxVelMPS);
        

        // Using SDS 6.75 ratio
        public static final double kThrottleRotToWheelRot = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double kThrottleWheelRotToMeters = 1 / (Math.PI * kWheelDiameterMeters);
        
        public static final InvertedValue kThrottleInversion = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kAzimuthInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kThrottleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kAzimuthNeutralMode = NeutralModeValue.Brake;

        public static final AbsoluteSensorRangeValue kAzimuthEncoderRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue kAzimuthEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    }

    public static final class LimelightConstants {
        public static final String kLimelightName = "limelight";
    }

    public static final class DriveConstants {
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(0.0);
        public static final Pose2d STARTING_POSE = new Pose2d(
                0.5,
                3.0,
                STARTING_ROTATION);
        // private static final double MAX_ACCEL_DISTANCE = 6.0 * Math.pow(TIMEOUT_S, 2);
        private static final double MAX_ACCEL_DISTANCE = 0.01;
        private static final double GYRO_ERROR_DEGREES_TIMEOUT = (0.4 / 60) * kCANTimeoutMs;
        public static final double CONFIDENCE_TO_ERROR = 1.0;
        public static final Matrix<N3, N1> STATE_STDEVS_MATRIX = new MatBuilder<N3, N1>(
                Nat.N3(),
                Nat.N1())
                .fill(MAX_ACCEL_DISTANCE, MAX_ACCEL_DISTANCE, GYRO_ERROR_DEGREES_TIMEOUT);
        public static final Matrix<N3, N1> VISION_STDEVS_MATRIX = new MatBuilder<N3, N1>(
                Nat.N3(),
                Nat.N1())
                .fill(1.0, 1.0, 1.0);
    }

    public static final class IntakeConstants{
        public static final int kFrontMotorID = 16;
        public static final int kRearMotorID = 0;

        public static final int kCurrentLimit = 30;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final boolean kFrontInversion = false;
        public static final boolean kRearInversion = false;
    }

    public static final class ConveyorConstants {
        public static final int kMotorID = 0;

        public static final int kCurrentLimit = 30;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final boolean kInversion = false;
    }

    public static final class ShooterConstants{
        public static final int kLeftLeaderID = 0;
        public static final int kRightFollowerID = 0;
        public static final int kEncoderID = 0;

        // Motor Output Constants
        public static final InvertedValue kLeaderInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final boolean kFollowerOpposeLeaderInversion = true;
        public static final double kFollowerUpdateHz = 1000;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 60.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;

        // Position PID Constants
        public static final int kVelocityPIDSlot = 0;
        public static final double kV = 0.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;
        public static final double kErrorTolerance = 1.0;
    }

    public static final class AngleAdjusterConstants {
        public static final int kMotorID = 0;
        public static final int kEncoderID = 0;

        // Motor Output Constants
        public static final InvertedValue kInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 60.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;
        public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 0.01;

        // Motion Magic Constants
        public static final double kMotionMagicVelocity = 0.0;
        public static final double kMotionMagicAccel = 0.0;
        public static final double kMotionMagicJerk = 0.0;

        // Software Limit Switch Constants
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 0.0;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = 0.0;

        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
            kAimMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(20);
        static {
            kAimMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            // ...
        }
    }

    public static final class ArmConstants {
        public static final int kLeftLeaderID = 0;
        public static final int kRightFollowerID = 0;
        public static final int kEncoderID = 0;

        // Motor Output Constants
        public static final InvertedValue kLeaderInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final boolean kFollowerOpposeLeaderInversion = true;
        public static final double kFollowerUpdateHz = 1000;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 60.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;
        public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final GravityTypeValue kGravityCompensationType = GravityTypeValue.Arm_Cosine;
        public static final double kG = 0.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;
        
        public static final double kTolerance = 0.01;

        // Motion Magic Constants
        public static final double kMotionMagicVelocity = 0.0;
        public static final double kMotionMagicAccel = 0.0;
        public static final double kMotionMagicJerk = 0.0;

        // Software Limit Switch Constants
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 0.0;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = 0.0;
    }

    public static final class WristConstants {
        public static final int kMotorID = 0;
        public static final int kEncoderID = 0;

        // Motor Output Constants
        public static final InvertedValue kInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 60.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;
        public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 0.01;

        // Motion Magic Constants
        public static final double kMotionMagicVelocity = 0.0;
        public static final double kMotionMagicAccel = 0.0;
        public static final double kMotionMagicJerk = 0.0;

        // Software Limit Switch Constants
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 0.0;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = 0.0;
    }

    public static final class ClimbConstants {
        public static final int kLeftLeaderID = 0;
        public static final int kRightFollowerID = 0;

        // Motor Output Constants
        public static final InvertedValue kLeaderInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final boolean kFollowerOpposeLeaderInversion = true;
        public static final double kFollowerUpdateHz = 1000;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 60.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final GravityTypeValue kGravityCompensationType = GravityTypeValue.Elevator_Static;
        public static final double kG = 0.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        // Motion Magic Constants
        public static final double kMotionMagicVelocity = 0.0;
        public static final double kMotionMagicAccel = 0.0;
        public static final double kMotionMagicJerk = 0.0;

        // Software Limit Switch Constants
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 0.0;
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = 0.0;
    }

    public static final class SuperstructureConstants {
        // Arm
        public static final Rotation2d kArmStowAngle = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d kArmAmpAngle = Rotation2d.fromDegrees(1.0);

        // Wrist
        public static final Rotation2d kWristStowAngle = Rotation2d.fromDegrees(0.0);
        public static final Rotation2d kWristIntakingAngle = Rotation2d.fromDegrees(1.0);
        public static final Rotation2d kWristAmpAngle = Rotation2d.fromDegrees(2.0);
    }

    public static final class VisionConstants {
        public static final String NAME = "SmartDashboard";
        public static final String APRILTAGFAMILY = "tag16h5";
        private static final String APRILTAGFILENAME = "AprilTagPoses.json";
        public static final Path APRILTAGPATH = Filesystem.getDeployDirectory().toPath().resolve(APRILTAGFILENAME);
        private static final double CAMERAXDISPLACEMENT = 0.0772;
        private static final double CAMERAYDISPLACEMENT = 0.3429;
        // private static final double CAMERAZDISPLACEMENT = 0.56198;
        public static final Rotation2d[] CAMERAANGLES = { new Rotation2d(0), new Rotation2d(Math.PI) };

        public static final Pose2d[] CAMERALOCATIONS = {
                new Pose2d(CAMERAXDISPLACEMENT, -CAMERAYDISPLACEMENT, CAMERAANGLES[0]),
                new Pose2d(-CAMERAXDISPLACEMENT, -CAMERAYDISPLACEMENT, CAMERAANGLES[1]) };
        public static final Transform2d[] CAMERATRANSFORMS = { new Transform2d(CAMERALOCATIONS[0], new Pose2d()),
                new Transform2d(CAMERALOCATIONS[1], new Pose2d()) };

        public static final double ANGLEHISTSECS = 1.0;
        public static final double DISTANCETOLERANCE = 3.0;
        public static final double DISTANCEERRORFACTOR = 0.01;
        public static final double ANGLEERRORFACTOR = 1;
        // public static final Pose2d[] APRILTAG_POSE2DS = {new Pose2d(1, 1, new
        // Rotation2d(.5))};
        // public final Pose2d[] APRILTAG_POSE2DS =
        // JSONTools.GenerateAprilTagPoses(APRILTAGPATH);
        Path trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/");
        public static final int IMU_ID = 0;

        public static final double CONE_PIXELS_TO_METERS = 0.001;

        public static final double MID_FIELD_X_POS = 8.3;
        public static final double MID_FIELD_Y_POS = 4.2;
        
        public static final double ADD_VISION_TOLERANCE = 1.0;
        public static final double DISTANCE_RESET_TOLERANCE = 3.0;
        public static final double SPEED_RESET_TOLERANCE = 0.5;
        public static final double OMEGA_RESET_TOLERANCE = 0.2;
        
        public static final int NUM_THREADS = 10;
    }

    public static final class LightsConstants {
        public static final int CANDLE_ID = 0;
        public static final boolean LOS_BEHAVIOR = true;
        // public static final LEDStripType LED_STRIP_TYPE = LEDStripType.GRB;
        public static final double BRIGHTNESS_SCALAR = 1.0;
        public static final boolean STATUS_LED_CONFIG = false;
        // public static final VBatOutputMode V_BAT_OUTPUT_MODE = VBatOutputMode.Off;

        public static final int PRIMARY_ANIMATION_SLOT = 0;
    }

    public static final int kCANTimeoutMs = 10; 
    public static final int kLongCANTimeoutMs = 100; // constructors

    public static final String kCANBusName = "seCANdary";

    public static final double kMaxMotorVoltage = 12.0; 

    public static final boolean kEnableFOC = true; 

    public static final double kJoystickDeadband = 0.1;
}
