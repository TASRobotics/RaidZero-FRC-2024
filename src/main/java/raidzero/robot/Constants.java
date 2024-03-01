package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
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
        // public static final double kFrontLeftAzimuthOffset = -0.463379;
        // public static final double kFrontRightAzimuthOffset = -0.565674 + 0.5;
        // public static final double kRearLeftAzimuthOffset = -0.090088;
        // public static final double kRearRightAzimuthOffset = -0.674316 + 0.5;

        /* Alpha Offsets */
        // public static final double kFrontLeftAzimuthOffset = 0.192383;
        // public static final double kFrontRightAzimuthOffset = 0.397949 + 0.5;
        // public static final double kRearLeftAzimuthOffset = 0.255615;
        // public static final double kRearRightAzimuthOffset = -0.205811 + 0.5;

        /* Sheesh Offsets */
        public static final double kFrontLeftAzimuthOffset = -0.809814 /*+ 0.5*/;
        public static final double kFrontRightAzimuthOffset = -0.483643 /*+ 0.5*/;
        public static final double kRearLeftAzimuthOffset = -0.880859 /*+ 0.5*/;
        public static final double kRearRightAzimuthOffset = -0.961426 /*+ 0.5*/;

        public static final double kThrottleReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double kAzimuthReduction = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double kWheelDiameterMeters = 0.1016;

        public static final double kMaxVelMPS = 6000 * kThrottleReduction / 60 * Math.PI * 0.102;// 4.959668;
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
        public static final double kAzimuth_kP = 50.0;// .75
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

        public static final double kTranslationController_kP = 8.0; //5.0
        public static final double kTranslationController_kD = 0.0;
        public static final double kThetaController_kP = 2.5; //2.0
        public static final double kXControllerTolerance = 0.1; //0.1
        public static final double kYControllerTolerance = 0.1; //0.1
        public static final double kThetaControllerTolerance = Math.toRadians(5);

        public static final double kSnapController_kP = 0.1;
        public static final double kSnapController_kI = 0.0;
        public static final double kSnapController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kSnapControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularVelRPS, kMaxAngularVelRPS);
        public static final double kSnapControllerToleranceDegrees = 2.0;

        public static final double kAimAssistController_kP = 0.1;
        public static final double kAimAssistController_kI = 0.0;
        public static final double kAimAssistController_kD = 0.0;
        public static final TrapezoidProfile.Constraints kAimAssistControllerConstraints =
            new TrapezoidProfile.Constraints(kRealisticMaxVelMPS, kRealisticMaxVelMPS);


        // Using SDS 6.75 ratio
        public static final double kThrottleRotToWheelRot = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double kThrottleWheelRotToMeters = 1 / (Math.PI * kWheelDiameterMeters);
        public static final double kMetersToThrottleRot = kThrottleRotToWheelRot/kThrottleWheelRotToMeters;

        public static final InvertedValue kThrottleInversion = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kAzimuthInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kThrottleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kAzimuthNeutralMode = NeutralModeValue.Brake;

        public static final AbsoluteSensorRangeValue kAzimuthEncoderRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        public static final SensorDirectionValue kAzimuthEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

        public static CurrentLimitsConfigs kAzimuthCurrentLimitsConfigs = new CurrentLimitsConfigs();
        static {
            kAzimuthCurrentLimitsConfigs.SupplyCurrentLimit = 30;
            kAzimuthCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            kAzimuthCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
            kAzimuthCurrentLimitsConfigs.SupplyTimeThreshold = 0.2;
        }

        public static CurrentLimitsConfigs kThrottleCurrentLimitsConfigs = new CurrentLimitsConfigs();
        static {
            kThrottleCurrentLimitsConfigs.SupplyCurrentLimit = 40;
            kThrottleCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            kThrottleCurrentLimitsConfigs.SupplyCurrentThreshold = 60;
            kThrottleCurrentLimitsConfigs.SupplyTimeThreshold = 0.2;
        }

        public static final double kTeleopRampRate = 0.25;
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
        public static final int kFrontMotorID = 2;
        public static final int kRearMotorID = 1;

        public static final int kFrontCurrentLimit = 20;
        public static final int kRearCurrentLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final boolean kFrontInversion = true;
        public static final boolean kRearInversion = false;
    }

    public static final class ConveyorConstants {
        public static final int kMotorID = 11;

        public static final int kCurrentLimit = 30;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        public static final boolean kInversion = false;
    }

    public static final class ShooterConstants{
        public static final int kUpperLeaderID = 31;
        public static final int kLowerFollowerID = 32;

        // public static final double kTheoreticalMaxSpeedRPS = 6000.0 / 60;

        // Motor Output Constants
        public static final InvertedValue kLeaderInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final boolean kFollowerOpposeLeaderInversion = false;
        public static final double kFollowerUpdateHz = 1000;

        // Current Limit Constants
        // public static final double kSupplyCurrentLimit = 30.0;
        // public static final boolean kSupplyCurrentEnable = false;
        // public static final double kSupplyCurrentThreshold = 60.0;
        // public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;

        // Velocity PID Constants
        public static final int kVelocityPIDSlot = 0;
        public static final double kV = 0.12; // voltage/rps
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;
        public static final double kErrorTolerance = 1.0;
    }

    public static final class AngleAdjusterConstants {
        public static final int kMotorID = 51;
        public static final int kEncoderID = 21;

        // Motor Output Constants
        public static final InvertedValue kInversion = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 20.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 30.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = (260.0 / 18.0) * (25.0 / 1.0); // 1.0;
        public static final double kRotorToSensorRatio = (260.0 / 18.0) * (25.0 / 1.0);
        public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final double kV = 12.0 / (6000.0 / kRotorToSensorRatio / 60); // voltage/rps
        public static final double kP = 150.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 1.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double kTheoreticalMaxSpeedRPS = 6000.0 / kRotorToSensorRatio / 60;

        public static final double kMotionMagicVelocity = kTheoreticalMaxSpeedRPS * 0.60; // rotations per second
        public static final double kMotionMagicAccel = kTheoreticalMaxSpeedRPS * 10.0;
        public static final double kMotionMagicJerk = kTheoreticalMaxSpeedRPS * 100.0;

        // Software Limit Switch Constants
        // TODO
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 59.0 / 360.0; // 59.0 / 360.0; // rotations
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = 15.0 / 360.0; // rotations

        // Magnet Sensor Constants
        public static final SensorDirectionValue kSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final double kMagnetOffset = -0.142578 + 30.0 / 360.0; //-.077393 + 7.0 / 360; //-0.143311 + 30.0 / 180; // 0.499268 /// 42 deg
        public static final AbsoluteSensorRangeValue kAbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        // Aiming Constants
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kAimMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        static {
            // kAimMap.put(new InterpolatingDouble(ROBOT_DISTANCE_FROM_GOAL), new InterpolatingDouble(DESIRED_ANGLE));
            kAimMap.put(new InterpolatingDouble(1.124750566), new InterpolatingDouble(48.69140625));
            kAimMap.put(new InterpolatingDouble(1.630963346), new InterpolatingDouble(45.3515625));
            kAimMap.put(new InterpolatingDouble(2.345829831), new InterpolatingDouble(41.8359375));
            kAimMap.put(new InterpolatingDouble(2.928608228), new InterpolatingDouble(36.9140625));
            kAimMap.put(new InterpolatingDouble(3.399811629), new InterpolatingDouble(34.1015625));
            kAimMap.put(new InterpolatingDouble(3.878163152), new InterpolatingDouble(30.41015625));
            kAimMap.put(new InterpolatingDouble(4.144581513), new InterpolatingDouble(27.24609375));
            // ...
        }

        /**
         * Angle Adjuster Angle |    X Meters    |     Y Meters
         * 48.69140625 | 1.08 | 5.67
         * 45.3515625 | 1.57 | 5.82
         * 41.8359375 | 2.29 | 5.26
         * 36.9140625 | 2.88 | 5.30
         * 34.1015625 | 3.36 | 5.44
         * 30.41015625 | 3.84 | 5.57
         * 27.24609375 | 3.96 | 6.64
         * xxx | 4.20 | 7.00
         *
         * 1.124750566
            1.630963346
            2.345829831
            2.928608228
            3.399811629
            3.878163152
            4.144581513
         */

        public static final Rotation2d kMaxAngle = new Rotation2d(5);
    }

    public static final class ArmConstants {
        public static final int kLeftLeaderID = 11;
        public static final int kRightFollowerID = 12;
        public static final int kEncoderID = 11;

        // Motor Output Constants
        public static final InvertedValue kLeaderInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final boolean kFollowerOpposeLeaderInversion = true;
        public static final double kFollowerUpdateHz = 1000;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 30.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 40.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = 1.0;
        public static final double kRotorToSensorRatio = (125.0 / 1.0) * (62.0 / 44.0) * (42.0 / 39.0); // TODO
        public static final FeedbackSensorSourceValue kFeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        // public static final GravityTypeValue kGravityCompensationType = GravityTypeValue.Arm_Cosine;
        // public static final double kG = 0.0;
        public static final double kV = 12.0 / (6000.0 / kRotorToSensorRatio / 60.0);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 2.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double kTheoreticalMaxSpeedRPS = 6000.0 / kRotorToSensorRatio / 60.0;
        public static MotionMagicConfigs kUpMotionMagicConfigs = new MotionMagicConfigs();
        static {
            kUpMotionMagicConfigs.withMotionMagicCruiseVelocity(kTheoreticalMaxSpeedRPS * 1.0);
            kUpMotionMagicConfigs.withMotionMagicAcceleration(kTheoreticalMaxSpeedRPS * 3.0);
            kUpMotionMagicConfigs.withMotionMagicJerk(kTheoreticalMaxSpeedRPS * 40.0);
        }

        public static MotionMagicConfigs kDownMotionMagicConfigs = new MotionMagicConfigs();
        static {
            kDownMotionMagicConfigs.withMotionMagicCruiseVelocity(kTheoreticalMaxSpeedRPS * 1.0);
            kDownMotionMagicConfigs.withMotionMagicAcceleration(kTheoreticalMaxSpeedRPS * 1.5);
            kDownMotionMagicConfigs.withMotionMagicJerk(kTheoreticalMaxSpeedRPS * 10.0);
        }

        // Software Limit Switch Constants
        // TODO
        public static final boolean kForwardSoftLimitEnabled = true;
        public static final double kForwardSoftLimit = 90.0 / 360.0; // rotations
        public static final boolean kReverseSoftLimitEnabled = true;
        public static final double kReverseSoftLimit = -40.0 / 360.0; // rotations

        // Magnet Sensor Constants
        public static final SensorDirectionValue kSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final double kMagnetOffset = -0.439209;
        public static final AbsoluteSensorRangeValue kAbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    }

    public static final class WristConstants {
        public static final int kMotorID = 21;

        // public static final double kTheoreticalMaxSpeed = 600; // 6000*(1/25)(20/48)/60*360 degrees per second
        // public static final double kResetAngleRotations = 128.0 / 360.0;

        // Motor Output Constants
        public static final InvertedValue kInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Current Limit Constants
        public static final double kSupplyCurrentLimit = 40.0;
        public static final boolean kSupplyCurrentEnable = true;
        public static final double kSupplyCurrentThreshold = 50.0;
        public static final double kSupplyTimeThreshold = 0.2;

        // Feedback Constants
        public static final double kSensorToMechanismRatio = (25.0 / 1.0) * (48.0 / 20.0); // rotor rotations to wrist rotations

        // Position PID Constants
        public static final int kPositionPIDSlot = 0;
        public static final double kV = 12.0 / (6000.0 / kSensorToMechanismRatio / 60.0); // 12.0 V / max speed rps
        public static final double kP = 80.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPIDUpdateHz = 1000;

        public static final double kTolerance = 2.0 / 360.0; // rotations

        // Motion Magic Constants
        public static final double kTheoreticalMaxSpeedRPS = 6000.0 / kSensorToMechanismRatio / 60.0 * 10;
        // public static final double kTheoreticalMaxSpeedRPS = 100.0;
        public static final double kMotionMagicVelocity = kTheoreticalMaxSpeedRPS * 0.30;
        public static final double kMotionMagicAccel = kTheoreticalMaxSpeedRPS * 3.0;
        public static final double kMotionMagicJerk = kTheoreticalMaxSpeedRPS * 30.0;

        // Software Limit Switch Constants
        public static SoftwareLimitSwitchConfigs kNormalSoftLimits = new SoftwareLimitSwitchConfigs();
        static {
            kNormalSoftLimits.ForwardSoftLimitEnable = true;
            kNormalSoftLimits.ForwardSoftLimitThreshold = 280.0 / 360.0; // rotations
            kNormalSoftLimits.ReverseSoftLimitEnable = true;
            kNormalSoftLimits.ReverseSoftLimitThreshold = 0.0;
        }

        // Hardware Limit Switch Constants
        public static final ReverseLimitSourceValue kReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        public static final ReverseLimitTypeValue kReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        public static final boolean kReverseLimitEnabled = true; // check
        public static final boolean kReverseLimitAutosetPositionEnabled = true; // check
        public static final double kReverseLimitAutosetPositionValue = 0.0;
    }

    public static final class ClimbConstants {
        public static final int kLeftLeaderID = 41;
        public static final int kRightFollowerID = 42;

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
        // TODO
        public static final boolean kForwardSoftLimitEnabled = false;
        public static final double kForwardSoftLimit = 63.285156;
        public static final boolean kReverseSoftLimitEnabled = false;
        public static final double kReverseSoftLimit = 0.0;
    }

    public static final class SuperstructureConstants {
        // Arm
        public static final Rotation2d kArmStowAngle = Rotation2d.fromDegrees(-39.0);
        public static final Rotation2d kArmAmpAngle = Rotation2d.fromDegrees(75.0);
        public static final Rotation2d kArmTrapAngle = Rotation2d.fromDegrees(80.0);

        // Wrist
        public static final Rotation2d kWristStowAngle = Rotation2d.fromDegrees(10.0);
        public static final Rotation2d kWristIntakingAngle = Rotation2d.fromDegrees(120.0);
        public static final Rotation2d kWristAmpAngle = Rotation2d.fromDegrees(204.0);

        // Shooter
        public static final double kShootingSpeedRPS = 90.0;
    }

    public static final class VisionConstants {
        public static final String APRILTAG_CAM_NAME = "limelight";
        public static final double XY_STDS = 0.1; //0.1
        public static final double DEG_STDS = 1;

        public static final String NOTE_CAM_NAME = "limelight-object";
        public static final int NOTE_FILTER_SIZE = 5;

        public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d());
    }

    public static final int kCANTimeoutMs = 10;
    public static final int kLongCANTimeoutMs = 100; // constructors

    public static final String kCANBusName = "seCANdary";

    public static final double kMaxMotorVoltage = 12.0;

    public static final boolean kEnableFOC = true;

    public static final double kJoystickDeadband = 0.1;
}
