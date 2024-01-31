package raidzero.robot.auto.sequences;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;

public class ThreeNote extends AutoSequence {
    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("first note");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("2nd note");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("3rd note");
    private PathPlannerTrajectory trajectory3;
    //get 4th note
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("note 4");
    private PathPlannerTrajectory trajectory4;

    public ThreeNote() {
        Rotation2d test1 = new Rotation2d(Math.toRadians(0));
        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), test1);
        Rotation2d test2 = new Rotation2d(Math.toRadians(0)); 
        trajectory2 = path2.getTrajectory(new ChassisSpeeds(), test2);
        Rotation2d test3 = new Rotation2d(Math.toRadians(0)); 
        trajectory3 = path3.getTrajectory(new ChassisSpeeds(), test3);
        trajectory4 = path4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
    }

    @Override
    public void sequence() {
        List<Action> idk = new ArrayList();
        idk.add(new DrivePath(trajectory1));
        //idk.add(new WaitAction(1));
        idk.add(new DrivePath(trajectory2));
        //idk.add(new WaitAction(1));
        idk.add(new DrivePath(trajectory3));
        idk.add(new DrivePath(trajectory4));
        addAction(
            new SeriesAction(idk)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 note (with pause)";
    }
}
