package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ConveyorConstants;
import raidzero.robot.utils.requests.Request;

public class Conveyor extends Submodule {
    private static Conveyor instance = null;

    public static Conveyor getInstance() {
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    private Conveyor() {}

    private CANSparkMax mMotor = new CANSparkMax(ConveyorConstants.kMotorID, MotorType.kBrushless);

    public static class PeriodicIO {
        public double desiredPercentSpeed = 0.0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();
        mMotor.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        mMotor.setSmartCurrentLimit(ConveyorConstants.kCurrentLimit);
        mMotor.setIdleMode(ConveyorConstants.kIdleMode);
        mMotor.setInverted(ConveyorConstants.kInversion);
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {
        mMotor.set(mPeriodicIO.desiredPercentSpeed);
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {}

    public void setPercentSpeed(double speed) {
        mPeriodicIO.desiredPercentSpeed = speed;
    }

    public SparkLimitSwitch getLimitSwitch() {
        return mMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    }

    public Request conveyorRequest(double speed) {
        return new Request() {
            @Override
            public void act() {
                setPercentSpeed(speed);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }
}
