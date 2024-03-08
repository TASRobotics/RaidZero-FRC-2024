
package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Wrist;

public class ManualRunIntakeAction implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();

    private Timer timer = new Timer();

    private double mSpeed, mDuration;
    private boolean mWristDown;

    public ManualRunIntakeAction(boolean wristDown, double speed, double duration) {
        mSpeed = speed;
        mWristDown = wristDown;
        mDuration = duration;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(mDuration);
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        // mIntake.setPercentSpeed(mSpeed);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        mWrist.setAngle(mWristDown ? SuperstructureConstants.kWristIntakingAngle : SuperstructureConstants.kWristStowAngle);
        mIntake.setPercentSpeed(mSpeed);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        mIntake.setPercentSpeed(0.0);
    }
}
