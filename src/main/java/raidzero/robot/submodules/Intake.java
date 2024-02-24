package raidzero.robot.submodules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.utils.requests.Request;

public class Intake extends Submodule{
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {}

    private CANSparkMax mFrontMotor = new CANSparkMax(IntakeConstants.kFrontMotorID, MotorType.kBrushless);
    private CANSparkMax mRearMotor = new CANSparkMax(IntakeConstants.kRearMotorID, MotorType.kBrushless);

    private SparkLimitSwitch mBeamBreak = mFrontMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    public static class PeriodicIO {
        public double desiredFrontPercentSpeed = 0.0;
        public double desiredRearPercentSpeed = 0.0;

        public boolean limitTriggered = false;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mFrontMotor.restoreFactoryDefaults();
        mFrontMotor.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        mFrontMotor.setSmartCurrentLimit(IntakeConstants.kFrontCurrentLimit);
        mFrontMotor.setIdleMode(IntakeConstants.kIdleMode);
        mFrontMotor.setInverted(IntakeConstants.kFrontInversion);

        mRearMotor.restoreFactoryDefaults();
        mRearMotor.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        mRearMotor.setSmartCurrentLimit(IntakeConstants.kRearCurrentLimit);
        mRearMotor.setIdleMode(IntakeConstants.kIdleMode);
        mRearMotor.setInverted(IntakeConstants.kRearInversion);

        mBeamBreak.enableLimitSwitch(false);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        SmartDashboard.putNumber("Front Intake Current", mFrontMotor.getOutputCurrent());
        mPeriodicIO.limitTriggered = mBeamBreak.isPressed();
    }

    @Override
    public void run() {
        mFrontMotor.set(mPeriodicIO.desiredFrontPercentSpeed);
        mRearMotor.set(mPeriodicIO.desiredRearPercentSpeed);
    }

    @Override
    public void stop() {
        mFrontMotor.stopMotor();
        mRearMotor.stopMotor();
    }

    @Override
    public void zero() {}

    /**
     * Set intake percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double frontSpeed, double rearSpeed) {
        mPeriodicIO.desiredFrontPercentSpeed = frontSpeed;
        mPeriodicIO.desiredRearPercentSpeed = rearSpeed;
    }

    public void setPercentSpeed(double frontSpeed, double rearSpeed, boolean stopIfLimitIsTriggered) {
        if(stopIfLimitIsTriggered && mPeriodicIO.limitTriggered) {
            setPercentSpeed(0, 0);
        } else {
            setPercentSpeed(frontSpeed, rearSpeed);
        }
    }

    public boolean ringPresent() {
        return mPeriodicIO.limitTriggered;
    }

    public Request intakeRequest(double frontSpeed, double rearSpeed, boolean stopIfLimitIsTriggered, boolean waitUntilSettled) {
        return new Request() {
            @Override
            public void act() {
                setPercentSpeed(frontSpeed, rearSpeed, stopIfLimitIsTriggered);
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
}
