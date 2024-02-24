package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Superstructure;

public class ShootAction implements Action {
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Superstructure mAim = Superstructure.getInstance();
    private Timer timer = new Timer();

    private boolean mEnabled;

    public ShootAction(boolean on) {
        mEnabled = on;
    }

    @Override
    public boolean isFinished() {
        if(mEnabled) {
            return mShooter.isUpToSpeed();
        }
        return true;
        //return timer.hasElapsed(mDuration);
    }

    @Override
    public void start() {
        if(mEnabled) {
            mShooter.setVelocity(90);
        } else {
            mShooter.setPercentSpeed(0.0);
        }
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {}

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        // mIntake.setPercentSpeed(0);
    }
}