package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;

public class Intake extends Submodule{
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {}

    private CANSparkMax mLeader = new CANSparkMax(IntakeConstants.kLeaderID, MotorType.kBrushless);
    private CANSparkMax mFollower = new CANSparkMax(IntakeConstants.kFollowerID, MotorType.kBrushless);

    public static class PeriodicIO {
        public double desiredPercentSpeed = 0.0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mLeader.restoreFactoryDefaults();
        mLeader.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        mLeader.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        mLeader.setIdleMode(IntakeConstants.kIdleMode);

        mFollower.restoreFactoryDefaults();
        mFollower.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        mFollower.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        mFollower.setIdleMode(IntakeConstants.kIdleMode);

        mFollower.follow(mLeader);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        // SmartDashboard.putNumber("Intake current draw", mMotor.getOutputCurrent());
    }

    @Override
    public void run() {
        // if (Math.abs(mPercentOut) < 0.05) {
        //     holdPosition();
        // }
        // if (mControlState == ControlState.OPEN_LOOP) {
        //     mMotor.set(mPercentOut);
        //     mPrevOpenLoopPosition = mMotor.getPosition().getValue();
        // } else if (mControlState == ControlState.CLOSED_LOOP) {
            
        // }
        mLeader.set(mPeriodicIO.desiredPercentSpeed);
    }

    @Override
    public void stop() {
        mLeader.stopMotor();
        mFollower.stopMotor();
    }

    @Override
    public void zero() {}

    /**
     * Set intake percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    /** Hold position of intake */
    // public void holdPosition() {
    //     mControlState = ControlState.CLOSED_LOOP;
    //     if (Math.signum(mPercentOut) < 0)
    //         mDesiredPosition = mPrevOpenLoopPosition - 1;
    //     else
    //         mDesiredPosition = mPrevOpenLoopPosition + 1;
    // }
}
