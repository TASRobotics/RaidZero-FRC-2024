package raidzero.robot.auto.sequences;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.equation.Sequence;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.AngleShooterAction;
import raidzero.robot.auto.actions.AutomaticIntakeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.DrivePathNoteAim;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;
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

    }

    @Override
    public void sequence() {

        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory1), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(25.5))
                )), 
                new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePathNoteAim(trajectory2), 
                        new DrivePath(trajectory3)
                    )),
                    //new DrivePath(trajectory2), 
                    new AutomaticIntakeAction(5), 
                    new AngleShooterAction(Rotation2d.fromDegrees(  29))
                )), 
               // new DrivePath(trajectory3), //go to shoot place
                new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePathNoteAim(trajectory4), 
                        new DrivePath(trajectory5)
                    )),
                    new AutomaticIntakeAction(5), 
                    new AngleShooterAction(Rotation2d.fromDegrees(27.5))
                )), 
                //new DrivePath(trajectory5), //go to shoot place
                new RunConveyorAction(1.0, 0.5), // shoot 3rd note
                new ShootAction(false)
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 right blue";
    }
}
