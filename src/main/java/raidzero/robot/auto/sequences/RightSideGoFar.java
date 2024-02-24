package raidzero.robot.auto.sequences;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.submodules.Swerve;

public class RightSideGoFar extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("rsgf 1");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("rsgf 2");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("rsgf 3");
    private PathPlannerTrajectory trajectory3;
    private PathPlannerPath path4 = PathPlannerPath.fromPathFile("rsgf 4");
    private PathPlannerTrajectory trajectory4;
    private PathPlannerPath path5 = PathPlannerPath.fromPathFile("rsgf 5");
    private PathPlannerTrajectory trajectory5;

    public RightSideGoFar() {
        trajectory4 = path4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory5 = path5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory2 = path2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory3 = path3.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
    }

    @Override
    public void sequence() {
        List<Action> idk = new ArrayList();
        idk.add(new Res());
        idk.add(new DrivePath(trajectory1));
        idk.add(new DrivePath(trajectory2));
        idk.add(new DrivePath(trajectory3));
        idk.add(new DrivePath(trajectory4));
        idk.add(new DrivePath(trajectory5));
        addAction(
            new SeriesAction(idk)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "start right go diagonal";
    }
}
