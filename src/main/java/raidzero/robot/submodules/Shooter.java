package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.utils.requests.Request;

public class Shooter extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {}

    private TalonFX mOuterLeader = new TalonFX(ShooterConstants.kUpperLeaderID);
    private TalonFX mInnerFollower = new TalonFX(ShooterConstants.kLowerFollowerID);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private VelocityVoltage mVelocityVoltage = new VelocityVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(ShooterConstants.kVelocityPIDSlot)
        .withUpdateFreqHz(ShooterConstants.kPIDUpdateHz);

    private Follower mFollower = new Follower(mOuterLeader.getDeviceID(), ShooterConstants.kFollowerOpposeLeaderInversion);

    public static class PeriodicIO {
        public double desiredVelocity = 0.0;
        public double currentVelocity = 0.0;

        public double desiredPercentSpeed = 0.0;
        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mOuterLeader.getConfigurator().apply(getLeaderConfig(), Constants.kLongCANTimeoutMs);
        mInnerFollower.getConfigurator().apply(getFollowerConfig(), Constants.kLongCANTimeoutMs);

        // Follower follower = 
        mFollower.withUpdateFreqHz(ShooterConstants.kFollowerUpdateHz);
        mInnerFollower.setControl(mFollower);
    }

    @Override
    public void onStart(double timestamp) {
        mPeriodicIO.desiredVelocity = 0.0;
        mPeriodicIO.desiredPercentSpeed = 0.0;
    }

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentVelocity = mOuterLeader.getVelocity().refresh().getValueAsDouble();

        SmartDashboard.putNumber("left shooter current velocity", mOuterLeader.getVelocity().getValueAsDouble());
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
            mOuterLeader.setControl(mVelocityVoltage.withVelocity(mPeriodicIO.desiredVelocity));
            mInnerFollower.setControl(mFollower);
        } else if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mOuterLeader.setControl(mVoltageOut.withOutput(mPeriodicIO.desiredPercentSpeed * Constants.kMaxMotorVoltage));
            mInnerFollower.setControl(mFollower);
        }
    }

    @Override
    public void stop() {
        mOuterLeader.stopMotor();
        mInnerFollower.stopMotor();
    }

    @Override
    public void zero() {}

    public void setVelocity(double vel) {
        mPeriodicIO.controlState = ControlState.FEEDBACK;
        mPeriodicIO.desiredVelocity = vel;
    }

    public double getVelocity() {
        return mPeriodicIO.currentVelocity;
    }

    /**
     * Set shooter duty cycle
     * 
     * @param speed speed in [-1.0, 1.0]
     */
    public void setPercentSpeed(double speed) {
        mPeriodicIO.controlState = ControlState.FEEDFORWARD;
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    /**
     * Returns whether the shooter is up to the setpoint speed.
     * 
     * @return whether the shooter is up to speed
     */
    public boolean isUpToSpeed() {
        return Math.abs(mPeriodicIO.desiredVelocity) > 1
                && Math.abs(mPeriodicIO.currentVelocity - mPeriodicIO.desiredVelocity) < ShooterConstants.kErrorTolerance;
    }

    public Request shooterRequest(double velocity, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setVelocity(velocity);
            }

            @Override
            public boolean isFinished() {
                return waitUntilSettled ? isUpToSpeed() : true;
            }
        };
    }

    /** Configure intake motor & integrated encoder/PID controller */
    private TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(ShooterConstants.kLeaderInversion);
        motorOutputConfigs.withNeutralMode(ShooterConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        // currentLimitsConfigs.withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit);
        // currentLimitsConfigs.withSupplyCurrentLimitEnable(ShooterConstants.kSupplyCurrentEnable);
        // currentLimitsConfigs.withSupplyCurrentThreshold(ShooterConstants.kSupplyCurrentThreshold);
        // currentLimitsConfigs.withSupplyTimeThreshold(ShooterConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(ShooterConstants.kSensorToMechanismRatio);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKV(ShooterConstants.kV);
        slot0Configs.withKP(ShooterConstants.kP);
        slot0Configs.withKI(ShooterConstants.kI);
        slot0Configs.withKD(ShooterConstants.kD);
        config.withSlot0(slot0Configs);

        return config; 
    }

    private TalonFXConfiguration getFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withNeutralMode(ShooterConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        // currentLimitsConfigs.withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit);
        // currentLimitsConfigs.withSupplyCurrentLimitEnable(ShooterConstants.kSupplyCurrentEnable);
        // currentLimitsConfigs.withSupplyCurrentThreshold(ShooterConstants.kSupplyCurrentThreshold);
        // currentLimitsConfigs.withSupplyTimeThreshold(ShooterConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

        return config;
    }
}