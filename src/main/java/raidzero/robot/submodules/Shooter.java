package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.dashboard.Tab;

public class Shooter extends Submodule {

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
    }

    private TalonFX motorLeft;
    private TalonFX motorRight;
    final VelocityVoltage m_velocity = new VelocityVoltage(0);
    final VoltageOut m_request = new VoltageOut(0);
    private double outputPercentSpeed = 0.0;

    @Override
    public void onInit() {
        motorLeft = new TalonFX(ShooterConstants.kMotorLeftID, Constants.kCANBusName);
        motorLeft.getConfigurator().apply(getMotorLeftConfig(),Constants.kLongCANTimeoutMs);

        Follower follower = new Follower(ShooterConstants.kMotorLeftID,true);
        motorRight.getConfigurator().apply(getMotorRightConfig(),Constants.kLongCANTimeoutMs);
        motorRight = new TalonFX(ShooterConstants.kMotorRightID, Constants.kCANBusName);
        motorRight.setControl(follower);
        zero();
    }

    @Override
    public void onStart(double timestamp) {
        outputPercentSpeed = 0.0;
        zero();
    }

    @Override
    public void update(double timestamp) {
        SmartDashboard.putNumber("left shooter current velocity", motorLeft.getVelocity().getValueAsDouble());
    }

    @Override
    public void run() {
        if (Math.abs(outputPercentSpeed) < 0.1) {
            stop();
        } else {
            m_velocity.Slot = 0;
            motorLeft.setControl(m_velocity.withVelocity(outputPercentSpeed * ShooterConstants.FAKE_MAX_SPEED));
        }
    }

    @Override
    public void stop() {
        outputPercentSpeed = 0.0;
        motorLeft.setControl(m_request.withOutput(12.0));
    }

    @Override
    public void zero() {
        motorLeft.setPosition(0.0, Constants.TIMEOUT_MS);
    }

    /**
     * Fires up the shooter.
     * 
     * @param percentSpeed speed of the shooter in [-1.0, 1.0]
     * @param freeze       whether to disregard the speed and keep the previous speed
     */
    public void shoot(double percentSpeed, boolean freeze) {
        if (freeze) {
            return;
        }
        outputPercentSpeed = percentSpeed;
    }

    /**
     * Returns whether the shooter is up to the setpoint speed.
     * 
     * @return whether the shooter is up to speed
     */
    public boolean isUpToSpeed() {
        return Math.abs(outputPercentSpeed) > 0.1
                && Math.abs(motorLeft.getClosedLoopError().getValueAsDouble()) < ShooterConstants.ERROR_TOLERANCE;
    }
    /** Configure intake motor & integrated encoder/PID controller */
    private TalonFXConfiguration getMotorLeftConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(ShooterConstants.kMotorLeftInversion);
        motorOutputConfigs.withNeutralMode(ShooterConstants.kMotorLeftNeutralMode);
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
    }

    private TalonFXConfiguration getMotorRightConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        //motorOutputConfigs.withInverted(ShooterConstants.kMotorRightInversion);
        motorOutputConfigs.withNeutralMode(ShooterConstants.kMotorRightNeutralMode);
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
    }
}