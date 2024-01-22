package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ArmConstants;

public class Arm extends Submodule {
    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {}

    private CANSparkMax mLeftLeader = new CANSparkMax(ArmConstants.kLeftLeaderID, ArmConstants.kMotorType);
    private CANSparkMax mRightFollower = new CANSparkMax(ArmConstants.kRightFollowerID, ArmConstants.kMotorType);

    private RelativeEncoder mEncoder = mLeftLeader.getEncoder();
    private SparkPIDController mArmController = mLeftLeader.getPIDController();
    private TrapezoidProfile mArmMotionProfile = new TrapezoidProfile(ArmConstants.kConstraints);

    public static class PeriodicIO {
        public double desiredPosition = 0.0;
        public double currentPosition = 0.0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();


    @Override
    public void onInit() {
        configureArmMotors(mLeftLeader, mRightFollower);
        configureArmController(mArmController, mEncoder);
    }

    @Override
    public void onStart(double timestamp) {
        zero();
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
    public void stop() {}

    @Override
    public void zero() {
        mEncoder.setPosition(0.0);
    }

    private void configureArmMotors(CANSparkMax leader, CANSparkMax follower) {
        leader.restoreFactoryDefaults();
        leader.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        leader.setIdleMode(ArmConstants.kMotorIdleMode);
        leader.setInverted(ArmConstants.kInversion);
        leader.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        leader.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kForwardSoftLimit);
        leader.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kReverseSoftLimit);

        follower.restoreFactoryDefaults();
        follower.enableVoltageCompensation(Constants.kMaxMotorVoltage);
        follower.setIdleMode(ArmConstants.kMotorIdleMode);
        follower.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        follower.follow(leader, ArmConstants.kFollowerInversion);
    }

    private void configureArmController(SparkPIDController controller, RelativeEncoder encoder) {
        encoder.setPositionConversionFactor(0.0);
        encoder.setVelocityConversionFactor(0.0);

        controller.setFeedbackDevice(encoder);
        controller.setFF(0, ArmConstants.kPIDSlot);
        controller.setP(0, ArmConstants.kPIDSlot);
        controller.setI(0, ArmConstants.kPIDSlot);
        controller.setD(0, ArmConstants.kPIDSlot);
    }
}
