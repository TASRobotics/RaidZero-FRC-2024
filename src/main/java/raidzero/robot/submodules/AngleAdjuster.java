package raidzero.robot.submodules;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import raidzero.robot.Constants;
import raidzero.robot.Constants.AngleAdjusterConstants;

public class AngleAdjuster extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static AngleAdjuster instance = null;

    public static AngleAdjuster getInstance() {
        if (instance == null) {
            instance = new AngleAdjuster();
        }
        return instance;
    }

    private AngleAdjuster() {}

    private TalonFX mMotor = new TalonFX(AngleAdjusterConstants.kMotorID, Constants.kCANBusName);
    private CANcoder mEncoder = new CANcoder(AngleAdjusterConstants.kEncoderID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(AngleAdjusterConstants.kPositionPIDSlot)
        .withUpdateFreqHz(AngleAdjusterConstants.kPIDUpdateHz);

    public static class PeriodicIO {
        
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        // TODO Auto-generated method stub
        super.onInit();
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
        // TODO Auto-generated method stub
        super.run();
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
}
