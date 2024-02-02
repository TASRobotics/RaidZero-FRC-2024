package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class Res implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    public Res( ) {
       
    }

    @Override
    public boolean isFinished() {
        swerve.first();
        return true;
    }

    @Override
    public void start() {
        swerve.first();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("finish res");
    }
}
