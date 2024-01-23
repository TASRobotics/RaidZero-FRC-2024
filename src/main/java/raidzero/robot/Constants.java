package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {
    /**
     * Swerve Constants
     */
    public static final class SwerveConstants {
        public static final double kOpenLoopRampRate = 0.25;
        public static final double kClosedLoopRampRate = 0.0;
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

        public static final double kFrontLeftAzimuthOffset = -0.463379;
        public static final double kFrontRightAzimuthOffset = -0.565674 + 0.5;
        public static final double kRearLeftAzimuthOffset = -0.090088;
        public static final double kRearRightAzimuthOffset = -0.674316 + 0.5;



        public static final double kThrottleReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double kAzimuthReduction = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double kWheelDiameterMeters = 0.1016;

        public static final double kMaxVelMPS = 4.959668;
        public static final double kRealisticMaxVelMPS = 4.2;

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

        /** 254 Pathing Constants (smooth): */
        public static final double kMaxDriveVelMPS = kMaxVelMPS * 0.6;
        public static final double kMaxDriveAccelMPSPS = kMaxDriveVelMPS * 1.25;
        public static final double kMaxAngularVelRPS = 1.2 * Math.PI;
        public static final double kMaxAngularAccelRPSPS = kMaxAngularVelRPS * 2;
        /** 254 Pathing Constants (fast): */
        // public static final double MAX_DRIVE_VEL = MAX_VEL_MPS;
        // public static final double MAX_DRIVE_ACCEL = MAX_DRIVE_VEL / 0.2;
        // public static final double MAX_STEERING_VEL = Units.degreesToRadians(1000);

        /** 254 Module Constants */
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

        /** 1678 Pathing Constants */
        public static final double kTranslationController_kP = 0.5; //1.0
        public static final double kThetaController_kP = 0;
        public static final double kXControllerTolerance = 0.1;
        public static final double kYControllerTolerance = 0.1;
        public static final double kThetaControllerTolerance = Math.toRadians(5);

        /** AutoAim Constants */
        public static final double kAAXController_kP = 1.6;
        public static final double kAAYController_kP = 1.6;
        public static final double kAAThetaController_kP = 1.0;
        public static final double kAAThetaController_kD = 0.1;
        public static final double kAAXControllerTolerance = 0.01;
        public static final double kAAYControllerTolerance = 0.01;
        public static final double kAAThetaControllerTolerance = Math.toRadians(0.2);

        // Using SDS 6.75 ratio
        public static final double kThrottleRotToWheelRot = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double kThrottleWheelRotToMeters = 1 / (Math.PI * kWheelDiameterMeters);
        
        public static final InvertedValue kThrottleInversion = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kAzimuthInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kThrottleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kAzimuthNeutralMode = NeutralModeValue.Brake;

        public static final AbsoluteSensorRangeValue kAzimuthEncoderRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue kAzimuthEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;



        public static final boolean kRotorInvertSensorPhase = true;
        /** Current Limits */
        // public static final SupplyCurrentLimitConfiguration kRotorCurrentLimit = new SupplyCurrentLimitConfiguration(
        //         true,
        //         25,
        //         40,
        //         0.1);
        // public static final SupplyCurrentLimitConfiguration kThrottleCurrentLimit = new SupplyCurrentLimitConfiguration(
        //         true,
        //         35,
        //         60,
        //         0.1);
    }

    public static final class DriveConstants {
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(0.0);
        public static final Pose2d STARTING_POSE = new Pose2d(
                0.5,
                3.0,
                STARTING_ROTATION);
        // private static final double MAX_ACCEL_DISTANCE = 6.0 * Math.pow(TIMEOUT_S, 2);
        private static final double MAX_ACCEL_DISTANCE = 0.01;
        private static final double GYRO_ERROR_DEGREES_TIMEOUT = (0.4 / SECONDS_IN_MINUTE) * TIMEOUT_S;
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

    public class PathingConstants {
        public static final int BASE_TRAJ_PERIOD_MS = 0;
        public static final int MIN_POINTS_IN_TALON = 10;
        public static final int TRANSMIT_PERIOD_MS = 20;
    }

    public static final class IntakeConstants{
        public static final int kMotorID = 9;
        public static final InvertedValue kMotorInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Brake;
    }

    public static final class ShooterConstants{
        public static final int kMotorRightID = 10;
        public static final int kMotorLeftID = 11;
        public static final InvertedValue kMotorLeftInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kMotorLeftNeutralMode = NeutralModeValue.Brake;
        public static final InvertedValue kMotorRightInversion = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kMotorRightNeutralMode = NeutralModeValue.Brake;
        public static final double FAKE_MAX_SPEED = 50;
        public static final double ERROR_TOLERANCE = 10;
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

        /**
         * Auto Alignment Constants
         */
        // Blue Alliance
        // public static final Pose2d BLL = new Pose2d(1.66, 4.57, Rotation2d.fromDegrees(155));
        // public static final Pose2d BLM = new Pose2d(1.85, 4.66, Rotation2d.fromDegrees(180));
        // public static final Pose2d BLR = new Pose2d(1.85, 4.08, Rotation2d.fromDegrees(180));
        // public static final Pose2d BML = new Pose2d(1.85, 3.53, Rotation2d.fromDegrees(180));
        // public static final Pose2d BMM = new Pose2d(1.85, 2.94, Rotation2d.fromDegrees(180));
        // public static final Pose2d BMR = new Pose2d(1.85, 2.34, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRL = new Pose2d(1.85, 1.90, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRM = new Pose2d(1.85, 1.36, Rotation2d.fromDegrees(180));
        // public static final Pose2d BRR = new Pose2d(1.85, 0.52, Rotation2d.fromDegrees(180));
        // // Red Alliance
        // public static final Pose2d RLL = new Pose2d(14.79, 0.97, Rotation2d.fromDegrees(-22));
        // public static final Pose2d RLM = new Pose2d(14.65, 0.94, Rotation2d.fromDegrees(0));
        // public static final Pose2d RLR = new Pose2d(14.65, 1.53, Rotation2d.fromDegrees(0));
        // public static final Pose2d RML = new Pose2d(14.65, 2.12, Rotation2d.fromDegrees(0));
        // public static final Pose2d RMM = new Pose2d(14.65, 2.64, Rotation2d.fromDegrees(0));
        // public static final Pose2d RMR = new Pose2d(14.65, 3.25, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRL = new Pose2d(14.65, 3.82, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRM = new Pose2d(14.65, 4.26, Rotation2d.fromDegrees(0));
        // public static final Pose2d RRR = new Pose2d(14.65, 4.72, Rotation2d.fromDegrees(0));
        // // Human Pickup Station
        // public static final Pose2d BL_LOAD = new Pose2d(15.73, 7.67, Rotation2d.fromDegrees(0));
        // public static final Pose2d BR_LOAD = new Pose2d(15.73, 5.99, Rotation2d.fromDegrees(0));
        // public static final Pose2d RL_LOAD = new Pose2d(0.79, 5.99, Rotation2d.fromDegrees(180));
        // public static final Pose2d RR_LOAD = new Pose2d(0.79, 7.67, Rotation2d.fromDegrees(180));

        //Blue Alliance
        public static final double BLL = 5.20;
        public static final double BLM = 4.66;
        public static final double BLR = 4.08;
        public static final double BML = 3.53;
        public static final double BMM = 2.94;
        public static final double BMR = 2.34;
        public static final double BRL = 1.90;
        public static final double BRM = 1.36;
        public static final double BRR = 0.52;
        
        //Red Alliance
        public static final double RLL = 1.03;
        public static final double RLM = 0.99;
        public static final double RLR = 1.46;
        public static final double RML = 2.17;
        public static final double RMM = 2.64;
        public static final double RMR = 3.23;
        public static final double RRL = 3.79;
        public static final double RRM = 4.26;
        public static final double RRR = 4.77;
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

    public static final String NETWORKTABLESNAME = "SmartDashboard";

    public static final int TIMEOUT_MS = 20;
    public static final double TIMEOUT_S = TIMEOUT_MS / 1000.0f;
    public static final int SECONDS_IN_MINUTE = 60;
    public static final double SQRTTWO = Math.sqrt(2);
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final double VOLTAGE_COMP = 12.0;

    public static final int kCANTimeoutMs = 10; 
    public static final int kLongCANTimeoutMs = 100; // constructors

    public static final String kCANBusName = "seCANdary";

    public static final double kMaxMotorVoltage = 12.0; 

    public static final boolean kEnableFOC = true; 

    public static final double kJoystickDeadband = 0.1;
}
