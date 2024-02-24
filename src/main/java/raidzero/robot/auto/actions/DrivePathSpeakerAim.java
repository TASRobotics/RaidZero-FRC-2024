package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import raidzero.robot.submodules.Swerve;

public class DrivePathSpeakerAim implements Action {
    private static final Swerve swerve = Swerve.getInstance();

    private PathPlannerTrajectory mTrajectory;

    public DrivePathSpeakerAim(PathPlannerTrajectory trajectory) {
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
        swerve.followPath(mTrajectory);
    }

    @Override
    public void update() {
        swerve.setPathingSpeakerAim(true);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        swerve.stop();
        swerve.setPathingSpeakerAim(false);
    }
}
