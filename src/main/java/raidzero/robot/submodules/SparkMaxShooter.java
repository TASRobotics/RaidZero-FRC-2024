package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMaxShooter extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static SparkMaxShooter instance = null;

    public static SparkMaxShooter getInstance() {
        if (instance == null) {
            instance = new SparkMaxShooter();
        }
        return instance;
    }

    private SparkMaxShooter() {}

    private CANSparkMax mLeftLeader = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax mRightFollower = new CANSparkMax(1, MotorType.kBrushless);
    private SparkPIDController mController = mLeftLeader.getPIDController();
    private RelativeEncoder mEncoder = mLeftLeader.getEncoder();

    public static class PeriodicIO {
        public double desiredPercentOutput = 0.0;
        public double desiredVelocityRPM = 0.0;
        public double currentVelocityRPM = 0.0;

        public ControlState controlState = ControlState.FEEDFORWARD;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();


    @Override
    public void onInit() {
        // TODO Auto-generated method stub
        super.onInit();
    }

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {
        mPeriodicIO.currentVelocityRPM = mEncoder.getVelocity();
    }

    @Override
    public void run() {
        if(mPeriodicIO.controlState == ControlState.FEEDFORWARD) {
            mController.setReference(mPeriodicIO.desiredPercentOutput, ControlType.kDutyCycle);
        } else if(mPeriodicIO.controlState == ControlState.FEEDBACK) {
            // perhaps smart velocity???
            mController.setReference(mPeriodicIO.desiredVelocityRPM, ControlType.kVelocity);
        }
    }

    @Override
    public void stop() {
        mLeftLeader.stopMotor();
        mRightFollower.stopMotor();
    }

    @Override
    public void zero() {
        mEncoder.setPosition(0.0);
    }

    private void setMotorConfig(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.enableVoltageCompensation(0.0);
        motor.setControlFramePeriodMs(0);
        motor.setSmartCurrentLimit(0);
    }

    private void setPIDConfig(SparkPIDController controller) {

    }

    private void setEncoderConfig(RelativeEncoder encoder) {

    }
}
