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
import raidzero.robot.auto.actions.AngleShooterAction;
import raidzero.robot.auto.actions.AutomaticIntakeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.DrivePathNoteAim;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.ManualRunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.submodules.Swerve;

public class ThreeNoteRed extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("Copy of first note");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path1p5 = PathPlannerPath.fromPathFile("Copy of first.5 note");
    private PathPlannerTrajectory trajectory1p5;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("Copy of 2nd note");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("Copy of 3rd note");
    private PathPlannerTrajectory trajectory3;
    //get 4th note
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("Copy of 4th note");
    private PathPlannerTrajectory trajectory4;
    PathPlannerPath path5 = PathPlannerPath.fromPathFile("Copy of 4th note go shoot");
    private PathPlannerTrajectory trajectory5;
    //get 5th note

    public ThreeNoteRed() {
        Rotation2d test1 = new Rotation2d(Math.toRadians(0)); 
        // Rotation2d test1 = path1.getPoint(0).rotationTarget.getTarget();
        // Rotation2d test1 = mSwerve.getPose().getRotation();
        trajectory1p5 = path1p5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));

        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), test1);
        Rotation2d test2 = new Rotation2d(Math.toRadians(0)); 
        trajectory2 = path2.getTrajectory(new ChassisSpeeds(), test2);
        Rotation2d test3 = new Rotation2d(Math.toRadians(0)); 
        trajectory3 = path3.getTrajectory(new ChassisSpeeds(), test3);
        trajectory4 = path4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory5 = path5.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        
    }

    @Override
    public void sequence() {
        List<Action> idk = new ArrayList();
        idk.add(new Res());
        idk.add(new DrivePath(trajectory1));
        idk.add(new DrivePath(trajectory1p5));
        //idk.add(new ParallelAction(new LinkedList<>(Arrays.asList(new DrivePath(trajectory2), new RunIntakeAction(1)))));
        idk.add(new DrivePath(trajectory2));
        //idk.add(new WaitAction(1));
        idk.add(new DrivePath(trajectory3));
        idk.add(new DrivePath(trajectory4));
        idk.add(new DrivePath(trajectory5));
        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory1), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(40))
                )), 
                new RunConveyorAction(1.0, 1.0), 
                new ParallelAction(Arrays.asList(
                    new DrivePathNoteAim(trajectory1p5), 
                    new AutomaticIntakeAction(1.0), 
                    new AngleShooterAction(Rotation2d.fromDegrees(40))
                )), 
                // new DrivePath(trajectory2), 
                new RunConveyorAction(1.0, 1.0)
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 close 1 mid";
    }
}
