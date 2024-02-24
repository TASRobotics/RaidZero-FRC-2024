package raidzero.robot.auto.actions;

import raidzero.robot.submodules.Superstructure;

public class AutomaticIntakeAction implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private boolean done = false;

    public AutomaticIntakeAction() {
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        done = mSuperstructure.intakeChoreographed(true);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        mSuperstructure.intakeChoreographed(false);
    }
}
