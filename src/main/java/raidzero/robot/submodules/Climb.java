package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ClimbConstants;

public class Climb extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static Climb instance = null;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {}

    private TalonFX mLeftLeader = new TalonFX(ClimbConstants.kLeftLeaderID);
    private TalonFX mRightFollower = new TalonFX(ClimbConstants.kRightFollowerID);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(ClimbConstants.kPositionPIDSlot)
        .withUpdateFreqHz(ClimbConstants.kPIDUpdateHz);

    private Follower mFollower = new Follower(mLeftLeader.getDeviceID(), ClimbConstants.kFollowerOpposeLeaderInversion);

    public static class PeriodicIO {
        public double desiredPosition = 0.0;
        public double currentPosition = 0.0;

        public double desiredPercentSpeed = 0.0;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mLeftLeader.getConfigurator().apply(getLeaderConfig(), Constants.kLongCANTimeoutMs);
        mRightFollower.getConfigurator().apply(getFollowerConfig(), Constants.kLongCANTimeoutMs);

        // Follower follower = new Follower(mLeftLeader.getDeviceID(), ClimbConstants.kFollowerOpposeLeaderInversion);
        mFollower.withUpdateFreqHz(ClimbConstants.kFollowerUpdateHz);
        mRightFollower.setControl(mFollower);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentPosition = mLeftLeader.getPosition().refresh().getValueAsDouble();
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
            mLeftLeader.setControl(mMotionMagicVoltage.withPosition(mPeriodicIO.desiredPosition));
            mRightFollower.setControl(mFollower);
        } else if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mLeftLeader.setControl(mVoltageOut.withOutput(mPeriodicIO.desiredPercentSpeed * Constants.kMaxMotorVoltage));
            mRightFollower.setControl(mFollower);
        }
    }

    @Override
    public void stop() {
        mLeftLeader.stopMotor();
        mRightFollower.stopMotor();
    }

    @Override
    public void zero() {
        mLeftLeader.setPosition(0.0);
    }

    public void setPosition(double position) {
        mPeriodicIO.controlState = ControlState.FEEDBACK;
        mPeriodicIO.desiredPosition = position;
    }

    public double getPosition() {
        return mPeriodicIO.currentPosition;
    }

    public void setPercentSpeed(double speed) {
        mPeriodicIO.controlState = ControlState.FEEDFORWARD;
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    private TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(ClimbConstants.kLeaderInversion);
        motorOutputConfigs.withNeutralMode(ClimbConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(ClimbConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(ClimbConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(ClimbConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(ClimbConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(ClimbConstants.kSensorToMechanismRatio);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withGravityType(ClimbConstants.kGravityCompensationType);
        slot0Configs.withKG(ClimbConstants.kG);
        slot0Configs.withKP(ClimbConstants.kP);
        slot0Configs.withKI(ClimbConstants.kI);
        slot0Configs.withKD(ClimbConstants.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(ClimbConstants.kMotionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(ClimbConstants.kMotionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(ClimbConstants.kMotionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitEnable(ClimbConstants.kForwardSoftLimitEnabled);
        softLimitConfigs.withForwardSoftLimitThreshold(ClimbConstants.kForwardSoftLimit);
        softLimitConfigs.withReverseSoftLimitEnable(ClimbConstants.kReverseSoftLimitEnabled);
        softLimitConfigs.withReverseSoftLimitThreshold(ClimbConstants.kReverseSoftLimit);
        config.withSoftwareLimitSwitch(softLimitConfigs);

        return config;
    }

    private TalonFXConfiguration getFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(ClimbConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(ClimbConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(ClimbConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(ClimbConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(ClimbConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

        return config;
    }
}
