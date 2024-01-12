package raidzero.robot.submodules;

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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;

public class SwerveModule extends Submodule implements Sendable {

    private enum ControlState {
        VELOCITY, PATHING, TESTING, PERCENT
    };

    private TalonFX mThrottle; 
    private TalonFX mAzimuth; 

    private CANcoder mAzimuthEncoder; 

    private VoltageOut throttleVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private VoltageOut azimuthVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private PositionVoltage azimuthPositionVoltage = new PositionVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(SwerveConstants.kAzimuthPositionPIDSlot);


    public static class PeriodicIO {
        public SwerveModuleState currentState = new SwerveModuleState(); 
        public SwerveModuleState desiredState = new SwerveModuleState();

        public SwerveModulePosition currentPosition = new SwerveModulePosition();

        public double outputThrottlePercentSpeed = 0.0;
        public double outputAzimuthPercentSpeed = 0.0; 

        public ControlState controlState = ControlState.VELOCITY;
    }


    private PeriodicIO mPeriodicIO = new PeriodicIO();

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

        mAzimuthEncoder = new CANcoder(azimuthEncoderId, Constants.kCANBusName);
        mAzimuthEncoder.getConfigurator().apply(getAzimuthEncoderConfig(forwardAngle), Constants.kLongCANTimeoutMs);

        mAzimuth = new TalonFX(azimuthId, Constants.kCANBusName);
        mAzimuth.getConfigurator().apply(getAzimuthConfig(), Constants.kLongCANTimeoutMs);

        mPeriodicIO.currentState = new SwerveModuleState();
        mPeriodicIO.desiredState = new SwerveModuleState();
        mPeriodicIO.currentPosition = new SwerveModulePosition(); 

        stop();
    }

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        mPeriodicIO.controlState = ControlState.PERCENT;
        mPeriodicIO.outputThrottlePercentSpeed = 0.0;
        mPeriodicIO.outputAzimuthPercentSpeed = 0.0;
        resetToAbsolute();
    }

    /**
     * Reads cached inputs & calculate outputs.
     */
    @Override
    public void update(double timestamp) {
        Rotation2d wrappedRotation = Rotation2d.fromRadians(mAzimuthEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);

        mPeriodicIO.currentState = new SwerveModuleState(
            mThrottle.getVelocity().getValue(), 
            wrappedRotation
        );

        // mCurrentPosition = new SwerveModulePosition(
        //     mThrottle.getPosition().getValue(), 
        //     Rotation2d.fromRadians(mAzimuth.getPosition().getValue() * Math.PI)
        // );

        mPeriodicIO.currentPosition = new SwerveModulePosition(
            mThrottle.getPosition().getValue(), 
            wrappedRotation
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
        // return -mCurrentState.angle.getDegrees();
        return mAzimuthEncoder.getAbsolutePosition().getValue();
    }

    @Override
    public void zero() {
        resetToAbsolute();
    }

    /** Runs components in the submodule that have continuously changing inputs. */
    public void run() {
        switch (mPeriodicIO.controlState) {
            case VELOCITY:
                mThrottle.setControl(new VelocityVoltage(mPeriodicIO.desiredState.speedMetersPerSecond));
                mAzimuth.setControl(new PositionVoltage(mPeriodicIO.desiredState.angle.getRadians() / (Math.PI * 2)));
                break;
            case PATHING:
                break;
            case TESTING:
                // mThrottle.setControl(throttleVoltageOut.withOutput(mPeriodicIO.outputThrottlePercentSpeed * Constants.kMaxMotorVoltage));
                // mAzimuth.setControl(azimuthVoltageOut.withOutput(mPeriodicIO.outputAzimuthPercentSpeed * Constants.kMaxMotorVoltage));
                break;
            case PERCENT:
                // mThrottle.setControl(new VoltageOut(mDesiredState.speedMetersPerSecond * Constants.kMaxMotorVoltage));
                // mAzimuth.setControl(new PositionVoltage(mPeriodicIO.desiredState.angle.getRadians() / Math.PI).withSlot(0));
                // mAzimuth.setControl(azimuthPositionVoltage.withPosition(mPeriodicIO.desiredState.angle.getRadians() / (Math.PI * 2)));
                mAzimuth.setControl(azimuthPositionVoltage.withPosition(getDesiredAzimuthOutputAngle(mPeriodicIO.desiredState.angle.getDegrees() / 360)));
                break;
        }
    }

    public void resetToAbsolute() {
        mAzimuth.setPosition(mAzimuthEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setClosedLoopState(SwerveModuleState state) {
        mPeriodicIO.controlState = ControlState.VELOCITY;

        state = SwerveModuleState.optimize(state, mPeriodicIO.currentState.angle);

        mPeriodicIO.desiredState = state;
    }

    public void setOpenLoopState(SwerveModuleState state) {
        mPeriodicIO.controlState = ControlState.PERCENT;
        state = SwerveModuleState.optimize(state, mPeriodicIO.currentState.angle);
        mPeriodicIO.desiredState = state; 
    }

    public SwerveModuleState getState() {
        return mPeriodicIO.currentState;
    }

    public SwerveModulePosition getPosition() {
        return mPeriodicIO.currentPosition;
    }

    public TalonFX getAzimuthMotor() {
        return mAzimuth;
    }

    public TalonFX getThrottleMotor() {
        return mThrottle; 
    }

    public PeriodicIO getPeriodicIO () {
        return mPeriodicIO;
    }

    /**
     * Get corrected Azimuth angle
     * 
     * @param angle desired angle to turn to (units: rotations)
     * @return corrected angle
     */
    public double getDesiredAzimuthOutputAngle(double angle) {
        double currentAngle = mAzimuthEncoder.getPosition().getValueAsDouble();
        double delta = angle - currentAngle;
        delta = delta % 1.0;
        while (delta > 0.5)
            delta -= 1.0;
        while (delta < -0.5)
            delta += 1.0;

        return currentAngle + delta;
    }

    public void turnToAngleTest(boolean yes) {
        mPeriodicIO.controlState = ControlState.TESTING;
        if(yes)
            mAzimuth.setControl(azimuthPositionVoltage.withPosition(0.75));
        else 
            mAzimuth.setControl(azimuthPositionVoltage.withPosition(0));
    }

    public void testThrottleAndRotor(double throttleOutput, double rotorOutput) {
        mPeriodicIO.controlState = ControlState.TESTING;

        mPeriodicIO.outputThrottlePercentSpeed = throttleOutput;
        mPeriodicIO.outputAzimuthPercentSpeed = rotorOutput;
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
        // feedbackConfigs.withFeedbackRemoteSensorID(mAzimuthEncoder.getDeviceID());
        // feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
        // feedbackConfigs.withRotorToSensorRatio(1 / SwerveConstants.kAzimuthReduction);
        feedbackConfigs.withSensorToMechanismRatio(1 / SwerveConstants.kAzimuthReduction);
        config.withFeedback(feedbackConfigs);

        // Position PID Configuration
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.withKP(SwerveConstants.kAzimuth_kP);
        slot0Configs.withKI(SwerveConstants.kAzimuth_kI);
        slot0Configs.withKD(SwerveConstants.kAzimuth_kD);
        config.withSlot0(slot0Configs);

        // Closed Loop General Configs
        // NO WRAP
        // ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        // closedLoopGeneralConfigs.ContinuousWrap = true; 
        // config.withClosedLoopGeneral(closedLoopGeneralConfigs);

        return config; 
    }

    private CANcoderConfiguration getAzimuthEncoderConfig(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Magnet Sensor Configuration
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.withAbsoluteSensorRange(SwerveConstants.kAzimuthEncoderRange);
        // magnetSensorConfigs.withMagnetOffset(0);
        magnetSensorConfigs.withSensorDirection(SwerveConstants.kAzimuthEncoderDirection);
        config.withMagnetSensor(magnetSensorConfigs);

        return config;
    }
}
