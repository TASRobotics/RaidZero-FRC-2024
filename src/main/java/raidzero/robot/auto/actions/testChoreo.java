package raidzero.robot.auto.actions;

import com.choreo.lib.*;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import raidzero.robot.submodules.Swerve;

public class TestChoreo implements Action{

    private ChoreoTrajectory traj = Choreo.getTrajectory("NewPath");
    private static final Swerve swerve = Swerve.getInstance();

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
        swerve.followPath(traj);
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