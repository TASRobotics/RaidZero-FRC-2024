package raidzero.robot.submodules;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.AngleAdjusterConstants;
import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.requests.ParallelRequest;
import raidzero.robot.utils.requests.Request;
import raidzero.robot.utils.requests.SequentialRequest;
import raidzero.robot.utils.requests.WaitRequest;

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
    // private static final Climb mClimb = Climb.getInstance();
    private static final Intake mIntake = Intake.getInstance();
    // private static final Limelight mLimelight = Limelight.getInstance();
    // private static final Shooter mShooter = Shooter.getInstance();
    // private static final Swerve mSwerve = Swerve.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();
    private static final Conveyor mConveyor = Conveyor.getInstance();

    private static final Vision mVision = Vision.getInstance();

    private Request mActiveRequest = null;
    // private ArrayList<Request> mQueuedRequests = new ArrayList<>(0);
    private boolean mHasNewRequest = false;
    private boolean mAllRequestsComplete = false;

    private SuperstructureState mCurrentState;

    private Alliance mAlliance;

    private boolean mBeamBreakWasToggled = false;
    private boolean mNoteHasPassed = false;

    @Override
    public void onInit() {}

    @Override
    public void onStart(double timestamp) {
        clearRequestQueue();
        mAlliance = DriverStation.getAlliance().get();
    }

    @Override
    public void update(double timestamp) {
        SmartDashboard.putBoolean("Note passed beam break", noteHasPassedBeamBreak());
        SmartDashboard.putBoolean("Note at beam break", mIntake.ringPresent());
    }

    @Override
    public void run() {
        try {
            if(mHasNewRequest && mActiveRequest != null) {
                mActiveRequest.act();
                mHasNewRequest = false;
            }
            if(mActiveRequest != null) {
                if(mActiveRequest.isFinished()) {
                    mActiveRequest = null;
                }
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

    public SuperstructureState getState() {
        return mCurrentState;
    }

    public void stowState() {
        request(new SequentialRequest(
            mIntake.intakeRequest(0.0, 0.0, false, false),
            new ParallelRequest(
                mWrist.wristRequest(SuperstructureConstants.kWristStowAngle, true), 
                mArm.armRequest(SuperstructureConstants.kArmStowAngle, true)
            )            
        ));
        mCurrentState = SuperstructureState.IDLE;
    }

    public void ampState() {
        request(new ParallelRequest(
            mArm.armRequest(SuperstructureConstants.kArmAmpAngle, true),
            new SequentialRequest(
                new WaitRequest(0.05), 
                mWrist.wristRequest(SuperstructureConstants.kWristAmpAngle, true)
            )
        ));
        mCurrentState = SuperstructureState.AMP;
    }

    public void angleShooter() {
        if(mVision.getSpeakerDistance(mAlliance) == 0.0) {
            return;
        }
        double dist = mVision.getSpeakerDistance(mAlliance);
        double desiredAngleDegrees = 0.0;
        if(AngleAdjusterConstants.kAimMap.getInterpolated(new InterpolatingDouble(dist)).value != null) {
            desiredAngleDegrees = AngleAdjusterConstants.kAimMap.getInterpolated(new InterpolatingDouble(dist)).value.doubleValue();
        }

        SmartDashboard.putNumber("Desired Angle Val", desiredAngleDegrees);
        
        mAngleAdjuster.setAngle(Rotation2d.fromDegrees(desiredAngleDegrees));
    }

    public boolean intakeChoreographed(boolean enable) {
        if(enable && !mNoteHasPassed) {
            mIntake.setPercentSpeed(1.0, 1.0);
            mConveyor.setPercentSpeed(0.40);
            mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
        } else if(enable && mNoteHasPassed) {
            mIntake.setPercentSpeed(0.0, 0.0);
            mConveyor.setPercentSpeed(0.0);
            mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
            return true;
        } else if(!enable) {
            mNoteHasPassed = false;
            mConveyor.setPercentSpeed(0.0);
            mIntake.setPercentSpeed(0.0, 0.0);
            mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
        }
        return false;
    }

    private boolean noteHasPassedBeamBreak() {
        boolean beamBreakIsToggled = mIntake.ringPresent();
        if(beamBreakIsToggled != mBeamBreakWasToggled) {
            mBeamBreakWasToggled = beamBreakIsToggled;
            if(!mBeamBreakWasToggled) {
                mNoteHasPassed = true;
                return true;
            }
        }
        return false;
    }

    // public void scoreState() {
    //     if(mCurrentState == SuperstructureState.AMP) {
    //         request(
    //             mIntake.intakeRequest(-1.0, -1.0, false, false)
    //         );
    //     } else if(mCurrentState == SuperstructureState.SHOOT) {
    //         request(new SequentialRequest(
    //             mWrist.wristRequest(SuperstructureConstants.kWristIntakingAngle, true), 
    //             new ParallelRequest(
    //                 mIntake.intakeRequest(1.0, 1.0, false, false), 
    //                 mConveyor.conveyorRequest(1.0)
    //             )
    //         ));
    //     }
    // }

    // public void shooterState() {
    //     double robotDistanceToTargetMeters = 0;
    //     Rotation2d aimAngle = 
    //         Rotation2d.fromDegrees(AngleAdjusterConstants.kAimMap.get(new InterpolatingDouble(robotDistanceToTargetMeters)).value);

    //     request(new ParallelRequest(
    //         mShooter.shooterRequest(SuperstructureConstants.kShootingSpeedRPS, true),
    //         mAngleAdjuster.angleAdjusterRequest(aimAngle, true)
    //     ));
    // }
}