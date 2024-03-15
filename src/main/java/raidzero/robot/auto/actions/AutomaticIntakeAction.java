package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Superstructure;

public class AutomaticIntakeAction implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private Timer timer = new Timer();
    private double mTimeSeconds;
    private boolean done = false;

    public AutomaticIntakeAction(double timelimit) {
        mTimeSeconds = timelimit;
    }

    public AutomaticIntakeAction() {
        mTimeSeconds = 5;
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(mTimeSeconds)) return true;
        return done;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        done = mSuperstructure.intakeChoreographed(true, true, false);
    }

    @Override
    public void done() {
        timer.stop();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        mSuperstructure.intakeChoreographed(false, true, false);
    }
}
