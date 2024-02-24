package raidzero.robot.auto.sequences;

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
import raidzero.robot.auto.actions.WaitAction;

public class ThreeNoteRight extends AutoSequence {
    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("right rush");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("right rush 2");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("right rush 3");
    private PathPlannerTrajectory trajectory3;
    //get 4th note
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("right rush 4");
    private PathPlannerTrajectory trajectory4;
    PathPlannerPath path5 = PathPlannerPath.fromPathFile("right rush 5");
    private PathPlannerTrajectory trajectory5;
    //get 5th note
    //PathPlannerPath path6 = PathPlannerPath.fromPathFile("right rush 6");
    //private PathPlannerTrajectory trajectory6;
    PathPlannerPath path7 = PathPlannerPath.fromPathFile("right rush under 7");
    private PathPlannerTrajectory trajectory7;
    PathPlannerPath path8 = PathPlannerPath.fromPathFile("right rush under 8");
    private PathPlannerTrajectory trajectory8;
    PathPlannerPath path9 = PathPlannerPath.fromPathFile("right rush under 9");
    private PathPlannerTrajectory trajectory9;
    PathPlannerPath path10 = PathPlannerPath.fromPathFile("right rush under 10");
    private PathPlannerTrajectory trajectory10;
    PathPlannerPath path11 = PathPlannerPath.fromPathFile("right rush under 11");
    private PathPlannerTrajectory trajectory11;
    PathPlannerPath path12 = PathPlannerPath.fromPathFile("right rush under 12");
    private PathPlannerTrajectory trajectory12;

    public ThreeNoteRight() {
        Rotation2d test1 = new Rotation2d(Math.toRadians(0));
        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), test1);
        Rotation2d test2 = new Rotation2d(Math.toRadians(0)); 
        trajectory2 = path2.getTrajectory(new ChassisSpeeds(), test2);
        Rotation2d test3 = new Rotation2d(Math.toRadians(0)); 
        trajectory3 = path3.getTrajectory(new ChassisSpeeds(), test3);
        trajectory4 = path4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory5 = path5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        //trajectory6 = path6.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory7 = path7.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory8 = path8.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory9 = path9.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory10 = path10.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory11 = path11.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory12 = path12.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
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
        //idk.add(new DrivePath(trajectory6));
        idk.add(new DrivePath(trajectory7));
        
        addAction(
            new SeriesAction(idk)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 right";
    }
}
