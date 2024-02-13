package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.utils.requests.Request;

// DONE (as of 1/24/2024)

public class Arm extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {}

    private TalonFX mLeftLeader = new TalonFX(ArmConstants.kLeftLeaderID, Constants.kCANBusName);
    private TalonFX mRightFollower = new TalonFX(ArmConstants.kRightFollowerID, Constants.kCANBusName);

    private CANcoder mEncoder = new CANcoder(ArmConstants.kEncoderID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(ArmConstants.kPositionPIDSlot)
        .withUpdateFreqHz(ArmConstants.kPIDUpdateHz);

    public static class PeriodicIO {
        public Rotation2d desiredPosition = new Rotation2d();
        public Rotation2d currentPosition = new Rotation2d();

        public double desiredPercentSpeed = 0.0;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mEncoder.getConfigurator().apply(getCANCoderConfig(), Constants.kLongCANTimeoutMs);

        mLeftLeader.getConfigurator().apply(getLeaderConfig(mEncoder), Constants.kLongCANTimeoutMs);
        mRightFollower.getConfigurator().apply(getFollowerConfig(), Constants.kLongCANTimeoutMs);
        
        Follower follower = new Follower(mLeftLeader.getDeviceID(), ArmConstants.kFollowerOpposeLeaderInversion);
        follower.withUpdateFreqHz(ArmConstants.kFollowerUpdateHz);
        mRightFollower.setControl(follower);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentPosition = Rotation2d.fromRotations(mLeftLeader.getPosition().refresh().getValueAsDouble());
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
            mLeftLeader.setControl(mMotionMagicVoltage.withPosition(mPeriodicIO.desiredPosition.getRotations()));
        } else if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mLeftLeader.setControl(mVoltageOut.withOutput(mPeriodicIO.desiredPercentSpeed * Constants.kMaxMotorVoltage));
        }
    }

    @Override
    public void stop() {
        mLeftLeader.stopMotor();
        mRightFollower.stopMotor();
    }

    @Override
    public void zero() {}

    public void setAngle(Rotation2d angle) {
        mPeriodicIO.controlState = ControlState.FEEDBACK;
        mPeriodicIO.desiredPosition = angle;
    }

    public Rotation2d getAngle() {
        return mPeriodicIO.currentPosition;
    }

    public void setPercentSpeed(double speed) {
        mPeriodicIO.controlState = ControlState.FEEDFORWARD;
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    public Request armRequest(Rotation2d angle, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return waitUntilSettled ? Math.abs(mLeftLeader.getClosedLoopError().refresh().getValueAsDouble()) < ArmConstants.kTolerance : true;
            }
        };
    }

    private TalonFXConfiguration getLeaderConfig(CANcoder encoder) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(ArmConstants.kLeaderInversion);
        motorOutputConfigs.withNeutralMode(ArmConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(ArmConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(ArmConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(ArmConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(ArmConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(ArmConstants.kSensorToMechanismRatio);
        feedbackConfigs.withFeedbackRemoteSensorID(encoder.getDeviceID());
        feedbackConfigs.withFeedbackSensorSource(ArmConstants.kFeedbackSensorSource);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.withGravityType(ArmConstants.kGravityCompensationType);
        // slot0Configs.withKG(ArmConstants.kG);
        slot0Configs.withKP(ArmConstants.kP);
        slot0Configs.withKI(ArmConstants.kI);
        slot0Configs.withKD(ArmConstants.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(ArmConstants.kMotionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(ArmConstants.kMotionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(ArmConstants.kMotionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitEnable(ArmConstants.kForwardSoftLimitEnabled);
        softLimitConfigs.withForwardSoftLimitThreshold(ArmConstants.kForwardSoftLimit);
        softLimitConfigs.withReverseSoftLimitEnable(ArmConstants.kReverseSoftLimitEnabled);
        softLimitConfigs.withReverseSoftLimitThreshold(ArmConstants.kReverseSoftLimit);
        config.withSoftwareLimitSwitch(softLimitConfigs);

        return config;
    }

    private TalonFXConfiguration getFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(ArmConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(ArmConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(ArmConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(ArmConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(ArmConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

        return config;
    }

    private CANcoderConfiguration getCANCoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Magnet Sensor Configuration
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withSensorDirection(ArmConstants.kSensorDirection);
        magnetSensorConfigs.withMagnetOffset(ArmConstants.kMagnetOffset);
        magnetSensorConfigs.withAbsoluteSensorRange(ArmConstants.kAbsoluteSensorRange);
        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }
}