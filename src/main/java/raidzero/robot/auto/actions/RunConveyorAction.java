package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Conveyor;

public class RunConveyorAction implements Action {
    private static final Conveyor mConveyor = Conveyor.getInstance();
    private Timer timer = new Timer();

    private double mSpeed, mTimeSeconds;

    public RunConveyorAction(double speed, double timeSeconds) {
        //mDuration = duration;
        mSpeed = speed;
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
        mConveyor.setPercentSpeed(mSpeed);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
        mConveyor.setPercentSpeed(0);
    }
}
