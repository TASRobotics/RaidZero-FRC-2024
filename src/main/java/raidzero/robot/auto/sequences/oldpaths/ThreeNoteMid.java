package raidzero.robot.auto.sequences.oldpaths;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.sequences.AutoSequence;

public class ThreeNoteMid extends AutoSequence {
    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("mid rush");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("mid rush 2");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("mid rush 3");
    private PathPlannerTrajectory trajectory3;
    private PathPlannerPath path4 = PathPlannerPath.fromPathFile("mid rush 4");
    private PathPlannerTrajectory trajectory4;
    private PathPlannerPath path5 = PathPlannerPath.fromPathFile("mid rush 5");
    private PathPlannerTrajectory trajectory5;
    private PathPlannerPath path6 = PathPlannerPath.fromPathFile("mid rush 6");
    private PathPlannerTrajectory trajectory6;
    private PathPlannerPath path1point5 = PathPlannerPath.fromPathFile("mid rush 1.5");
    private PathPlannerTrajectory trajectory1point5;

    public ThreeNoteMid() {
        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory2 = path2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory3 = path3.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory4 = path4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory5 = path5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory6 = path6.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory1point5 = path1point5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));

    }

    @Override
    public void sequence() {
        List<Action> idk = new ArrayList();
        idk.add(new Res());
        idk.add(new DrivePath(trajectory1));
        idk.add(new DrivePath(trajectory1point5));
        idk.add(new DrivePath(trajectory2));
        idk.add(new DrivePath(trajectory3));
        idk.add(new DrivePath(trajectory4));
        idk.add(new DrivePath(trajectory5));
        idk.add(new DrivePath(trajectory6));
        addAction(
            new SeriesAction(idk)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 mid";
    }
}
