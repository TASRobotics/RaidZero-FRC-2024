package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Superstructure;

public class AimAction implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private Timer timer = new Timer();

    private double mTimeSeconds;

    public AimAction(double timeSeconds) {
        mTimeSeconds = timeSeconds;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(mTimeSeconds);
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
        mSuperstructure.angleShooter();
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
    }
}