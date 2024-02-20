package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

import raidzero.robot.Constants;
import raidzero.robot.Constants.WristConstants;
import raidzero.robot.utils.requests.Request;

public class Wrist extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static Wrist instance = null;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private Wrist() {}

    private TalonFX mMotor = new TalonFX(WristConstants.kMotorID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(WristConstants.kPositionPIDSlot)
        .withUpdateFreqHz(WristConstants.kPIDUpdateHz);

    public static class PeriodicIO {
        public Rotation2d desiredPosition = new Rotation2d();
        public Rotation2d currentPosition = new Rotation2d();

        public double desiredPercentSpeed = 0.0;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mMotor.getConfigurator().apply(getConfig(), Constants.kLongCANTimeoutMs);
    }

    @Override
    public void onStart(double timestamp) {
        zero();
    }

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentPosition = Rotation2d.fromRotations(mMotor.getPosition().refresh().getValueAsDouble());
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
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
    public void zero() {
        // mMotor.setPosition(WristConstants.kResetAngleRotations);
        mMotor.setPosition(0.0);
    }

    public TalonFX getMotor() {
        return mMotor;
    }

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

    public Request wristRequest(Rotation2d angle, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return waitUntilSettled ? Math.abs(mMotor.getClosedLoopError().refresh().getValueAsDouble()) < WristConstants.kTolerance : true;
            }
        };
    }

    private TalonFXConfiguration getConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(WristConstants.kInversion);
        motorOutputConfigs.withNeutralMode(WristConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(WristConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(WristConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(WristConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(WristConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(WristConstants.kSensorToMechanismRatio);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.withKV(WristConstants.kV);
        slot0Configs.withKP(WristConstants.kP);
        slot0Configs.withKI(WristConstants.kI);
        slot0Configs.withKD(WristConstants.kD);
        config.withSlot0(slot0Configs);

        // Motion Magic Configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(WristConstants.kMotionMagicVelocity);
        motionMagicConfigs.withMotionMagicAcceleration(WristConstants.kMotionMagicAccel);
        motionMagicConfigs.withMotionMagicJerk(WristConstants.kMotionMagicJerk);
        config.withMotionMagic(motionMagicConfigs);

        // Software Limit Switch Configuration 
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        softLimitConfigs.withForwardSoftLimitEnable(WristConstants.kForwardSoftLimitEnabled);
        softLimitConfigs.withForwardSoftLimitThreshold(WristConstants.kForwardSoftLimit);
        softLimitConfigs.withReverseSoftLimitEnable(WristConstants.kReverseSoftLimitEnabled);
        softLimitConfigs.withReverseSoftLimitThreshold(WristConstants.kReverseSoftLimit);
        config.withSoftwareLimitSwitch(softLimitConfigs);

        // Hardware Limit Switch Configuration
        HardwareLimitSwitchConfigs hardwareLimitConfigs = new HardwareLimitSwitchConfigs();
        hardwareLimitConfigs.withForwardLimitSource(WristConstants.kForwardLimitSource);
        hardwareLimitConfigs.withForwardLimitType(WristConstants.kForwardLimitType);
        hardwareLimitConfigs.withForwardLimitEnable(WristConstants.kForwardLimitEnabled);
        hardwareLimitConfigs.withReverseLimitSource(WristConstants.kReverseLimitSource);
        hardwareLimitConfigs.withReverseLimitEnable(WristConstants.kReverseLimitEnabled);

        return config;
    }
}
