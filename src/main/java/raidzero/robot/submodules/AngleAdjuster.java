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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

import raidzero.robot.Constants;
import raidzero.robot.Constants.AngleAdjusterConstants;
import raidzero.robot.utils.requests.Request;

public class AngleAdjuster extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static AngleAdjuster instance = null;

    public static AngleAdjuster getInstance() {
        if (instance == null) {
            instance = new AngleAdjuster();
        }
        return instance;
    }

    private AngleAdjuster() {}

    private TalonFX mMotor = new TalonFX(AngleAdjusterConstants.kMotorID, Constants.kCANBusName);
    private CANcoder mEncoder = new CANcoder(AngleAdjusterConstants.kEncoderID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(AngleAdjusterConstants.kPositionPIDSlot)
        .withUpdateFreqHz(AngleAdjusterConstants.kPIDUpdateHz);

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

        mMotor.getConfigurator().apply(getMotorConfig(mEncoder), Constants.kLongCANTimeoutMs);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentPosition = Rotation2d.fromRotations(mMotor.getPosition().refresh().getValueAsDouble());
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
            if(mPeriodicIO.desiredPosition.getRotations() > AngleAdjusterConstants.kForwardSoftLimit) {
                mPeriodicIO.desiredPosition = Rotation2d.fromRotations(AngleAdjusterConstants.kForwardSoftLimit);
            }
            if(mPeriodicIO.desiredPosition.getRotations() < AngleAdjusterConstants.kReverseSoftLimit) {
                mPeriodicIO.desiredPosition = Rotation2d.fromRotations(AngleAdjusterConstants.kReverseSoftLimit);
            }
            mMotor.setControl(mMotionMagicVoltage.withPosition(mPeriodicIO.desiredPosition.getRotations()));
        } else if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mMotor.setControl(mVoltageOut.withOutput(mPeriodicIO.desiredPercentSpeed * Constants.kMaxMotorVoltage));
        }
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {}

    public void setPercentSpeed(double speed) {
        mPeriodicIO.controlState = ControlState.FEEDFORWARD;
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    public void setAngle(Rotation2d angle) {
        mPeriodicIO.controlState = ControlState.FEEDBACK;
        mPeriodicIO.desiredPosition = angle;
    }

    public Rotation2d getAngle() {
        return mPeriodicIO.currentPosition;
    }

    public Request angleAdjusterRequest(Rotation2d angle, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return waitUntilSettled ? Math.abs(mMotor.getClosedLoopError().refresh().getValueAsDouble()) < AngleAdjusterConstants.kTolerance : true;
            }
        };
    }

    private TalonFXConfiguration getMotorConfig(CANcoder encoder) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(AngleAdjusterConstants.kInversion);
        motorOutputConfigs.withNeutralMode(AngleAdjusterConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(AngleAdjusterConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(AngleAdjusterConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(AngleAdjusterConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(AngleAdjusterConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(AngleAdjusterConstants.kSensorToMechanismRatio);
        feedbackConfigs.withRotorToSensorRatio(AngleAdjusterConstants.kRotorToSensorRatio);
        feedbackConfigs.withFeedbackRemoteSensorID(encoder.getDeviceID());
        feedbackConfigs.withFeedbackSensorSource(AngleAdjusterConstants.kFeedbackSensorSource);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKV(AngleAdjusterConstants.kV);
        slot0Configs.withKP(AngleAdjusterConstants.kP);
        slot0Configs.withKI(AngleAdjusterConstants.kI);
        slot0Configs.withKD(AngleAdjusterConstants.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(AngleAdjusterConstants.kMotionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(AngleAdjusterConstants.kMotionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(AngleAdjusterConstants.kMotionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitEnable(AngleAdjusterConstants.kForwardSoftLimitEnabled);
        softLimitConfigs.withForwardSoftLimitThreshold(AngleAdjusterConstants.kForwardSoftLimit);
        softLimitConfigs.withReverseSoftLimitEnable(AngleAdjusterConstants.kReverseSoftLimitEnabled);
        softLimitConfigs.withReverseSoftLimitThreshold(AngleAdjusterConstants.kReverseSoftLimit);
        config.withSoftwareLimitSwitch(softLimitConfigs);

        return config;
    }

    private CANcoderConfiguration getCANCoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Magnet Sensor Configuration
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withSensorDirection(AngleAdjusterConstants.kSensorDirection);
        magnetSensorConfigs.withMagnetOffset(AngleAdjusterConstants.kMagnetOffset);
        magnetSensorConfigs.withAbsoluteSensorRange(AngleAdjusterConstants.kAbsoluteSensorRange);
        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }
}
