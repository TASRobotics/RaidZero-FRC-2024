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
import com.revrobotics.CANSparkBase.IdleMode;

import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;

public class Intake extends Submodule{
    private Intake() {
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    private ControlState mControlState = ControlState.OPEN_LOOP;

    private double mPercentOut = 0.0;
    private double mDesiredPosition = 0.0;
    private double mPrevOpenLoopPosition = 0.0;

    private TalonFX mMotor;

    @Override
    public void onInit() {
        mMotor = new TalonFX(IntakeConstants.kMotorID, Constants.kCANBusName);
        mMotor.getConfigurator().apply(getMotorConfig(),Constants.kLongCANTimeoutMs);
        zero();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
        // SmartDashboard.putNumber("Intake current draw", mMotor.getOutputCurrent());
    }

    @Override
    public void run() {
        if (Math.abs(mPercentOut) < 0.05) {
            holdPosition();
        }
        if (mControlState == ControlState.OPEN_LOOP) {
            mMotor.set(mPercentOut);
            mPrevOpenLoopPosition = mMotor.getPosition().getValue();
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            
        }
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {
    }

    /**
     * Set intake percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mControlState = ControlState.OPEN_LOOP;
        mPercentOut = speed;
    }

    /** Hold position of intake */
    public void holdPosition() {
        mControlState = ControlState.CLOSED_LOOP;
        if (Math.signum(mPercentOut) < 0)
            mDesiredPosition = mPrevOpenLoopPosition - 1;
        else
            mDesiredPosition = mPrevOpenLoopPosition + 1;
    }

    /** Configure intake motor & integrated encoder/PID controller */
    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(IntakeConstants.kMotorInversion);
        motorOutputConfigs.withNeutralMode(IntakeConstants.kMotorNeutralMode);
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
