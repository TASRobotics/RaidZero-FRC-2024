package raidzero.robot.submodules;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.AngleAdjusterConstants;
import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.requests.ParallelRequest;
import raidzero.robot.utils.requests.Request;
import raidzero.robot.utils.requests.SequentialRequest;

public class Superstructure extends Submodule {
    public enum SuperstructureState {
        IDLE, AMP, SHOOT, INTAKE
    }
    
    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {}

    private static final AngleAdjuster mAngleAdjuster = AngleAdjuster.getInstance();
    private static final Arm mArm = Arm.getInstance();
    private static final Climb mClimb = Climb.getInstance();
    private static final Intake mIntake = Intake.getInstance();
    private static final Limelight mLimelight = Limelight.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Swerve mSwerve = Swerve.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();
    private static final Conveyor mConveyor = Conveyor.getInstance();

    private Request mActiveRequest = null;
    // private ArrayList<Request> mQueuedRequests = new ArrayList<>(0);
    private boolean mHasNewRequest = false;
    private boolean mAllRequestsComplete = false;

    private SuperstructureState mCurrentState;

    @Override
    public void onInit() {}

    @Override
    public void onStart(double timestamp) {
        clearRequestQueue();
    }

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {
        try {
            if(mHasNewRequest && mActiveRequest != null) {
                mActiveRequest.act();
                mHasNewRequest = false;
            }
            if(mActiveRequest.isFinished()) {
                mActiveRequest = null;
            }
            // if (mActiveRequest == null) {
            //     if (mQueuedRequests.isEmpty()) {
            //         mAllRequestsComplete = true;
            //     } else {
            //         request(mQueuedRequests.remove(0));
            //     }
            // } else if (mActiveRequest.isFinished()) {
            //     mActiveRequest = null;
            // }
        }  catch (Exception e) {
            e.printStackTrace();
        }
        
        SmartDashboard.putBoolean("All reqs complete", mAllRequestsComplete);
    }

    @Override
    public void stop() {}

    @Override
    public void zero() {}

    public void request(Request request) {
        // if(mActiveRequest == null) {
            mActiveRequest = request;
            mHasNewRequest = true;
            mAllRequestsComplete = false;
        // }
    }

    private void clearRequestQueue() {
        // mQueuedRequests.clear();
    }

    private void setRequestQueue(List<Request> requests) {
        clearRequestQueue();
        // for(Request request : requests) {
        //     mQueuedRequests.add(request);
        // }
    }

    public void stowState() {
        request(new SequentialRequest(
            mIntake.intakeRequest(0.0, 0.0, false, false),
            mWrist.wristRequest(SuperstructureConstants.kWristStowAngle, true), 
            mArm.armRequest(SuperstructureConstants.kArmStowAngle, true)
        ));
        mCurrentState = SuperstructureState.IDLE;
    }

    // public void groundIntakeState() {
    //     if(!mIntake.ringPresent()) {
    //         request(new SequentialRequest(
    //             mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, false),
    //             mIntake.intakeRequest(1.0, 0.5, true, true), 
    //             mWrist.wristRequest(SuperstructureConstants.kWristStowAngle, true)
    //         ));
    //     }
    //     mCurrentState = SuperstructureState.INTAKE;
    // }
    public void groundIntakeState() {
        request(new SequentialRequest(
            mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true), 
            mIntake.intakeRequest(1.0, 0.0, false, false)
        ));
    }

    public void ampState() {
        request(new SequentialRequest(
            mArm.armRequest(SuperstructureConstants.kArmAmpAngle, true), 
            mWrist.wristRequest(SuperstructureConstants.kWristAmpAngle, true)
        ));
        mCurrentState = SuperstructureState.AMP;
    }

    public void scoreState() {
        if(mCurrentState == SuperstructureState.AMP) {
            request(
                mIntake.intakeRequest(-1.0, -1.0, false, false)
            );
        } else if(mCurrentState == SuperstructureState.SHOOT) {
            request(new SequentialRequest(
                mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true), 
                new ParallelRequest(
                    mIntake.intakeRequest(1.0, 1.0, false, false), 
                    mConveyor.conveyorRequest(1.0)
                )
            ));
        }
    }

    public void shooterState() {
        double robotDistanceToTargetMeters = 0;
        Rotation2d aimAngle = 
            Rotation2d.fromDegrees(AngleAdjusterConstants.kAimMap.get(new InterpolatingDouble(robotDistanceToTargetMeters)).value);

        request(new ParallelRequest(
            mShooter.shooterRequest(SuperstructureConstants.kShootingSpeedRPS, true),
            mAngleAdjuster.angleAdjusterRequest(aimAngle, true)
        ));
    }
}