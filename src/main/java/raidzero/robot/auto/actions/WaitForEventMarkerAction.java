package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;

import raidzero.robot.submodules.Swerve;

public class WaitForEventMarkerAction implements Action {
    private EventMarker mMarker;
    private static final Swerve mSwerve = Swerve.getInstance();

    /**
     * Action that blocks until event marker has been reached
     * 
     * @param trajectory  the path the robot is following
     * @param name        marker name
     * @param currentTime current time in the trajectory
     */
    public WaitForEventMarkerAction(PathPlannerPath trajectory, int index) {
        mMarker = trajectory.getEventMarkers().get(index);
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }

    @Override
    public boolean isFinished() {
        // if(mSwerve.)
        // return mMarker.shouldTrigger(mSwerve.getPose());
        return false;
    }
}
