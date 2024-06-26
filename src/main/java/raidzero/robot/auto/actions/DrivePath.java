package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Swerve swerve = Swerve.getInstance();

    private PathPlannerTrajectory mTrajectory;

    public DrivePath(PathPlannerTrajectory trajectory) {
        mTrajectory = trajectory;
    }

    @Override
    public boolean isFinished() {
        if (swerve.isFinishedPathing()) {
            System.out.println("[Auto] Path finished!");
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        swerve.enableTeleopRampRate(false);
        swerve.followPath(mTrajectory);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        swerve.stop();
    }
}
