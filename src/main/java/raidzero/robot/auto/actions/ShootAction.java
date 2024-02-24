/*
package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Shooter;

public class ShootAction implements Action {
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Superstructure mAim = Superstructure.getInstance();
    private Timer timer = new Timer();

    private double mSpeed;

    public ShootAction(double speed) {
        mSpeed = speed;
    }

    @Override
    public boolean isFinished() {
        return mShooter.ringPresent();
        //return timer.hasElapsed(mDuration);
    }

    @Override
    public void start() {
        mAim.angleShooter();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        mShooter.setPercentSpeed(mSpeed,-mSpeed,true);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        // mIntake.setPercentSpeed(0);
    }
}
*/