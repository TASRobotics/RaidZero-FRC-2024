package raidzero.robot.submodules;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.Sendable;

public class SwerveModule extends Submodule implements Sendable {

    private enum ControlState {
        VELOCITY, PATHING, TESTING, PERCENT
    };

    private TalonFX mThrottle; 
    private TalonFX mAzimuth; 

    private CANcoder mAzimuthEncoder; 

    private SwerveModuleState mCurrentState;
    private SwerveModuleState mDesiredState;
    private SwerveModulePosition mCurrentPosition;

    private double mOutputThrottlePercentSpeed;
    private double mOutputAzimuthPercentSpeed;

    private ControlState mControlState = ControlState.VELOCITY;

    @Override
    public void onInit() {
        throw new RuntimeException("Cannot initialize SwerveModule by itself.");
    }

    /**
     * Called once when the submodule is initialized.
     */
    public void onInit(int throttleId, int azimuthId, int azimuthEncoderId, double forwardAngle) {
        mThrottle = new TalonFX(throttleId, Constants.kCANBusName);
        mThrottle.getConfigurator().apply(getThrottleConfig(), Constants.kLongCANTimeoutMs);

        mAzimuth = new TalonFX(azimuthId, Constants.kCANBusName);
        mAzimuth.getConfigurator().apply(getAzimuthConfig(), Constants.kLongCANTimeoutMs);

        mAzimuthEncoder = new CANcoder(azimuthEncoderId, Constants.kCANBusName);
        mAzimuthEncoder.getConfigurator().apply(getAzimuthEncoderConfig(forwardAngle), Constants.kLongCANTimeoutMs);

        mCurrentState = new SwerveModuleState();
        mDesiredState = new SwerveModuleState();
        mCurrentPosition = new SwerveModulePosition(); 

        stop();
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {}

    /**
     * Reads cached inputs & calculate outputs.
     */
    @Override
    public void update(double timestamp) {
        mCurrentState = new SwerveModuleState(
            mThrottle.getVelocity().getValue(), 
            Rotation2d.fromRadians(mAzimuth.getPosition().getValue() * Math.PI)
        );

        mCurrentPosition = new SwerveModulePosition(
            mThrottle.getPosition().getValue(), 
            Rotation2d.fromRadians(mAzimuth.getPosition().getValue() * Math.PI)
        );
    }

    @Override
    public void stop() {
        testThrottleAndRotor(0, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getNegatedAzimuthAngle, null);
    }

    public Double getNegatedAzimuthAngle() {
        return -mCurrentState.angle.getDegrees();
    }

    @Override
    public void zero() {}

    /** Runs components in the submodule that have continuously changing inputs. */
    public void run() {
        switch (mControlState) {
            case VELOCITY:
                mThrottle.setControl(new VelocityVoltage(mDesiredState.speedMetersPerSecond));
                mAzimuth.setControl(new PositionVoltage(mDesiredState.angle.getRadians() / Math.PI));
                break;
            case PATHING:
                break;
            case TESTING:
                mThrottle.setControl(new VoltageOut(mOutputThrottlePercentSpeed * Constants.kMaxMotorVoltage));
                mAzimuth.setControl(new VoltageOut(mOutputAzimuthPercentSpeed * Constants.kMaxMotorVoltage));
                break;
            case PERCENT:
                mThrottle.setControl(new VoltageOut(mDesiredState.speedMetersPerSecond * Constants.kMaxMotorVoltage));
                mAzimuth.setControl(new PositionVoltage(mDesiredState.angle.getRadians() / Math.PI));
                break;
        }
    }

    public void setClosedLoopState(SwerveModuleState state) {
        mControlState = ControlState.VELOCITY;
        mDesiredState = state;
    }

    public void setOpenLoopState(SwerveModuleState state) {
        mControlState = ControlState.PERCENT;
        mDesiredState = state; 
    }

    public SwerveModuleState getState() {
        return mCurrentState;
    }

    public SwerveModulePosition getPosition() {
        return mCurrentPosition;
    }

    public void testThrottleAndRotor(double throttleOutput, double rotorOutput) {
        mControlState = ControlState.TESTING;

        mOutputThrottlePercentSpeed = throttleOutput;
        mOutputAzimuthPercentSpeed = rotorOutput;
    }

    private TalonFXConfiguration getThrottleConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(SwerveConstants.kThrottleInversion);
        motorOutputConfigs.withNeutralMode(SwerveConstants.kThrottleNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withSensorToMechanismRatio(0.0);
        config.withFeedback(feedbackConfigs);

        // Velocity PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKA(0.0);
        slot0Configs.withKV(0.0);
        slot0Configs.withKS(0.0);
        slot0Configs.withKP(0.0);
        slot0Configs.withKI(0.0);
        slot0Configs.withKD(0.0);
        slot0Configs.withKG(0.0);
        config.withSlot0(slot0Configs);

        // Closed Loop General Configs
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        config.withClosedLoopGeneral(closedLoopGeneralConfigs);

        return config; 

        // throttle.configFactoryDefault();
        // throttle.setInverted(SwerveConstants.MOTOR_INVERSION);
        // throttle.setNeutralMode(NeutralMode.Brake);
        // throttle.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // throttle.config_kF(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KF);
        // throttle.config_kP(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KP);
        // throttle.config_kD(SwerveConstants.THROTTLE_VELOCITY_PID_SLOT, SwerveConstants.THROTTLE_KD);
        // throttle.configVoltageCompSaturation(Constants.VOLTAGE_COMP);
        // throttle.enableVoltageCompensation(true);
        // throttle.configSupplyCurrentLimit(SwerveConstants.THROTTLE_CURRENT_LIMIT);

        // throttle.configOpenloopRamp(SwerveConstants.kOpenLoopRampRate);
        // throttle.configClosedloopRamp(SwerveConstants.kClosedLoopRampRate);
    }

    private TalonFXConfiguration getAzimuthConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(SwerveConstants.kAzimuthInversion);
        motorOutputConfigs.withNeutralMode(SwerveConstants.kAzimuthNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackRemoteSensorID(mAzimuthEncoder.getDeviceID());
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
        feedbackConfigs.withRotorToSensorRatio(0.0);
        config.withFeedback(feedbackConfigs);

        // Position PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(0.0);
        slot0Configs.withKI(0.0);
        slot0Configs.withKD(0.0);
        config.withSlot0(slot0Configs);

        // Closed Loop General Configs
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true; 
        config.withClosedLoopGeneral(closedLoopGeneralConfigs);

        return config; 


        // rotor.configFactoryDefault();
        // rotor.setInverted(SwerveConstants.ROTOR_INVERSION);
        // rotor.setNeutralMode(NeutralMode.Brake);
        // rotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        // rotor.setSensorPhase(SwerveConstants.ROTOR_INVERT_SENSOR_PHASE);
        // rotor.configRemoteFeedbackFilter(encoder, 0);
        // rotor.config_kP(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KP);
        // rotor.config_kI(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KI);
        // rotor.config_kD(SwerveConstants.ROTOR_POSITION_PID_SLOT, SwerveConstants.ROTOR_KD);
        // rotor.configVoltageCompSaturation(Constants.VOLTAGE_COMP);
        // rotor.enableVoltageCompensation(true);
        // rotor.configSupplyCurrentLimit(SwerveConstants.ROTOR_CURRENT_LIMIT);
    }

    private CANcoderConfiguration getAzimuthEncoderConfig(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Magnet Sensor Configuration
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        magnetSensorConfigs.withMagnetOffset(angleOffset);
        magnetSensorConfigs.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }
}
