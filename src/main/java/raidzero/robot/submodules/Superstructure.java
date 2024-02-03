package raidzero.robot.submodules;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.utils.requests.ParallelRequest;
import raidzero.robot.utils.requests.Request;
import raidzero.robot.utils.requests.SequentialRequest;

public class Superstructure extends Submodule {
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
    private ArrayList<Request> mQueuedRequests = new ArrayList<>(0);
    private boolean mHasNewRequest = false;
    private boolean mAllRequestsComplete = false;

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
            if (mActiveRequest == null) {
                if (mQueuedRequests.isEmpty()) {
                    mAllRequestsComplete = true;
                } else {
                    request(mQueuedRequests.remove(0));
                }
            } else if (mActiveRequest.isFinished()) {
                mActiveRequest = null;
            }
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
        mActiveRequest = request;
        mHasNewRequest = true;
        mAllRequestsComplete = false;
    }

    private void clearRequestQueue() {
        mQueuedRequests.clear();
    }

    private void setRequestQueue(List<Request> requests) {
        clearRequestQueue();
        for(Request request : requests) {
            mQueuedRequests.add(request);
        }
    }

    public void stowState() {
        request(new SequentialRequest(
            mIntake.intakeRequest(0.0, 0.0),
            mWrist.wristRequest(SuperstructureConstants.kWristStowAngle, true), 
            mArm.armRequest(SuperstructureConstants.kArmStowAngle, true)
        ));
    }

    public void groundIntakeState(boolean prepShoot) {
        if(prepShoot) {
            request(new SequentialRequest(
               mIntake.intakeRequest(1.0, 0.8), 
               mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true)
            ));
        } else {
            request(new SequentialRequest(
                mIntake.intakeRequest(1.0, 0.0), 
                mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true)
            ));
        }
    }

    public void shootState() {
        boolean noteLoaded = false;
        if(noteLoaded) {
            request(new SequentialRequest(
                new ParallelRequest(
                    mAngleAdjuster.angleAdjusterRequest(null, true), 
                    mShooter.shooterRequest(0)
                ), 
                mConveyor.conveyorRequest(1.0)
            ));
        } else {
            request(new ParallelRequest(
                new SequentialRequest(
                    mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true),
                    mIntake.intakeRequest(1.0, 0.8)
                ), 
                new ParallelRequest(
                    mAngleAdjuster.angleAdjusterRequest(null, true), 
                    mShooter.shooterRequest(0)
                ), 
                mConveyor.conveyorRequest(1.0)
            ));
        }
    }
}