package raidzero.robot.submodules;

import java.lang.constant.Constable;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ClimbConstants;
import raidzero.robot.Constants.SwerveConstants;

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

    private TalonFX mLeftLeader = new TalonFX(ClimbConstants.kLeftLeaderID, Constants.kCANBusName);
    private TalonFX mRightFollower = new TalonFX(ClimbConstants.kRightFollowerID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private PositionVoltage mPositionVoltage = new PositionVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(ClimbConstants.kPIDSlot)
        .withUpdateFreqHz(ClimbConstants.kPIDUpdateHz);

    public static class PeriodicIO {
        public double desiredPosition = 0.0;
        public double currentPosition = 0.0;

        public double percentSpeed = 0.0;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    @Override
    public void onInit() {
        mLeftLeader.getConfigurator().apply(getLeaderConfig(), Constants.kLongCANTimeoutMs);
    }

    @Override
    public void onStart(double timestamp) {
        // TODO Auto-generated method stub
        super.onStart(timestamp);
    }

    @Override
    public void update(double timestamp) {
        // TODO Auto-generated method stub
        super.update(timestamp);
    }

    @Override
    public void run() {
        
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zero() {
        // TODO Auto-generated method stub
        super.zero();
    }

    private TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(ClimbConstants.kInversion);
        motorOutputConfigs.withNeutralMode(ClimbConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(0.0);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(true);
        currentLimitsConfigs.withSupplyCurrentThreshold(1);
        currentLimitsConfigs.withSupplyTimeThreshold(0.0);
        config.withCurrentLimits(currentLimitsConfigs);
        
        // Feedback Configuration
        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        // feedbackConfigs.withSensorToMechanismRatio(ClimbConstants.kThrottleRotToWheelRot * ClimbConstants.kThrottleWheelRotToMeters);
        // config.withFeedback(feedbackConfigs);

        // // Velocity PID Configuration
        // Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.withKA(ClimbConstants.kThrottle_kA);
        // slot0Configs.withKV(ClimbConstants.kThrottle_kV);
        // slot0Configs.withKP(ClimbConstants.kThrottle_kP);
        // slot0Configs.withKI(ClimbConstants.kThrottle_kI);
        // slot0Configs.withKD(ClimbConstants.kThrottle_kD);
        // config.withSlot0(slot0Configs);

        // Closed Loop General Configs
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        config.withClosedLoopGeneral(closedLoopGeneralConfigs);

        return config;
    }
}
