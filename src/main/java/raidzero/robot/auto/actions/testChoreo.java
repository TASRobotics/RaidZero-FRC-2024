package raidzero.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.submodules.Swerve;

public class TestChoreo implements Action{

    private PathPlannerTrajectory traj = PathPlannerPath.fromChoreoTrajectory("NewPath").getTrajectory(new ChassisSpeeds(), new Rotation2d());
    private static final Swerve swerve = Swerve.getInstance();

    public TestChoreo(){
        
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