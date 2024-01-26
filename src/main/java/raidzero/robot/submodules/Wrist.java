package raidzero.robot.submodules;

import java.io.Console;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import raidzero.robot.Constants;
import raidzero.robot.Constants.WristConstants;

public class Wrist extends Submodule {
    private enum ControlState {
        FEEDBACK, FEEDFORWARD
    }

    private static Wrist instance = null;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private Wrist() {}

    private TalonFX mMotor = new TalonFX(WristConstants.kMotorID, Constants.kCANBusName);
    private CANcoder mEncoder = new CANcoder(WristConstants.kEncoderID, Constants.kCANBusName);

    private VoltageOut mVoltageOut = new VoltageOut(0.0).withEnableFOC(Constants.kEnableFOC);
    private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0)
        .withEnableFOC(Constants.kEnableFOC)
        .withSlot(WristConstants.kPositionPIDSlot)
        .withUpdateFreqHz(WristConstants.kPIDUpdateHz);

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
