package raidzero.robot.submodules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.utils.requests.Request;

public class Intake extends Submodule{
    private enum ControlState {
        FEEDFORWARD, TORQUE
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {}

    private TalonFX mMotor = new TalonFX(IntakeConstants.kMotorID);
    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private TorqueCurrentFOC mTorqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private static final Conveyor mConveyor = Conveyor.getInstance();

    private SparkLimitSwitch mBeamBreak;

    private Timer mBeamBreakTimer = new Timer();

    public static class PeriodicIO {
        public double desiredPercentSpeed = 0.0;

        public double desiredStatorCurrent = 0.0;

        public boolean limitTriggered = false;
        public boolean limitWasTriggered = false;
        public boolean limitWasJustTriggered = false;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mMotor.getConfigurator().apply(getConfig(), Constants.kLongCANTimeoutMs);

        mBeamBreak = mConveyor.getLimitSwitch();
        mBeamBreak.enableLimitSwitch(false);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        boolean beamBreak = mBeamBreak.isPressed();
        // if(beamBreak != mPeriodicIO.limitWasTriggered) {
        //     mPeriodicIO.limitWasTriggered = beamBreak;
        //     if(mPeriodicIO.limitWasTriggered) {

        //     }
        // }


        // if(mBeamBreak.isPressed() && !mPeriodicIO.limitWasJustTriggered) {
        //     mPeriodicIO.limitWasJustTriggered = true;
        // } else {
        //     mPeriodicIO.limitWasJustTriggered = false;
        // }
        if(beamBreak) {
            mBeamBreakTimer.start();
        } else {
            mBeamBreakTimer.reset();
        }
        // if(mPeriodicIO.limitWasJustTriggered) {
        //     mBeamBreakTimer.restart();
        //     mBeamBreakTimer.start();
        // }
        mPeriodicIO.limitTriggered = mBeamBreakTimer.get() > IntakeConstants.kBeamBrakeTime && mBeamBreak.isPressed();
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mMotor.setControl(mVoltageOut.withOutput(mPeriodicIO.desiredPercentSpeed * Constants.kMaxMotorVoltage));
        } else if(mPeriodicIO.controlState == ControlState.TORQUE) {
            mMotor.setControl(mTorqueCurrentFOC.withOutput(mPeriodicIO.desiredStatorCurrent));
        }
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {}

    /**
     * Set intake percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mPeriodicIO.controlState = ControlState.FEEDFORWARD;
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    public void setStatorCurrent(double current) {
        mPeriodicIO.controlState = ControlState.TORQUE;
        mPeriodicIO.desiredStatorCurrent = current;
    }

    // @Deprecated
    // public void setPercentSpeed(double frontSpeed, double rearSpeed, boolean stopIfLimitIsTriggered) {
    //     if(stopIfLimitIsTriggered && mPeriodicIO.limitTriggered && Math.abs(mEncoder.getPosition()) < 1) {
    //         setPercentSpeed(0.5, 0.5);
    //     } else if(stopIfLimitIsTriggered && mPeriodicIO.limitTriggered) {
    //         setPercentSpeed(0, 0);
    //     } else {
    //         setPercentSpeed(frontSpeed, rearSpeed);
    //     }
    // }

    public boolean ringPresent() {
        return mPeriodicIO.limitTriggered;
    }

    public Request intakeRequest(double speed, boolean stopIfLimitIsTriggered, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setPercentSpeed(speed);
            }

            @Override
            public boolean isFinished() {
                if(stopIfLimitIsTriggered && !mPeriodicIO.limitTriggered && waitUntilSettled) {
                    return false;
                }
                return true;
            }
        };
    }

    private TalonFXConfiguration getConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor Output Configuration
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.withInverted(IntakeConstants.kLeaderInversion);
        motorOutputConfigs.withNeutralMode(IntakeConstants.kNeutralMode);
        config.withMotorOutput(motorOutputConfigs);

        // Current Limit Configuration
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimit(IntakeConstants.kSupplyCurrentLimit);
        currentLimitsConfigs.withSupplyCurrentLimitEnable(IntakeConstants.kSupplyCurrentEnable);
        currentLimitsConfigs.withSupplyCurrentThreshold(IntakeConstants.kSupplyCurrentThreshold);
        currentLimitsConfigs.withSupplyTimeThreshold(IntakeConstants.kSupplyTimeThreshold);
        config.withCurrentLimits(currentLimitsConfigs);

        // Open Loop Ramp Rate Configuration
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.withVoltageOpenLoopRampPeriod(IntakeConstants.kVoltageRampRate);
        config.withOpenLoopRamps(openLoopRampsConfigs);
        
        return config; 
    }
}
