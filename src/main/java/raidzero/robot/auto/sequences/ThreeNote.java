package raidzero.robot.auto.sequences;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.AngleShooterAction;
import raidzero.robot.auto.actions.AutomaticIntakeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;
import raidzero.robot.submodules.Swerve;

public class ThreeNote extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("first note");
    private PathPlannerTrajectory trajectory1;
    private PathPlannerPath path1p5 = PathPlannerPath.fromPathFile("first.5 note");
    private PathPlannerTrajectory trajectory1p5;
    private PathPlannerPath path2 = PathPlannerPath.fromPathFile("2nd note");
    private PathPlannerTrajectory trajectory2;
    private PathPlannerPath path3 = PathPlannerPath.fromPathFile("3rd note");
    private PathPlannerTrajectory trajectory3;
    //get 4th note
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("4th note");
    private PathPlannerTrajectory trajectory4;
    PathPlannerPath path5 = PathPlannerPath.fromPathFile("4th note go shoot");
    private PathPlannerTrajectory trajectory5;
    //get 5th note
    PathPlannerPath path6 = PathPlannerPath.fromPathFile("5th note");
    private PathPlannerTrajectory trajectory6;
    PathPlannerPath path7 = PathPlannerPath.fromPathFile("5th note go shoot");
    private PathPlannerTrajectory trajectory7;

    public ThreeNote() {
        if(DriverStation.getAlliance().get() == Alliance.Red) {
            path1 = path1.flipPath();
            path1p5 = path1p5.flipPath();
            path2 = path2.flipPath();
            path3 = path3.flipPath();
            // path4.flipPath();
        }
        // path1.getPreviewStartingHolonomicPose();
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
        trajectory6 = path6.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));
        trajectory7 = path7.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)));

    }

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory1), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(45))
                )), 
                new RunConveyorAction(1.0, 1.0), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory1p5), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(37))
                )), 
                // new DrivePath(trajectory2), 
                new RunConveyorAction(1.0, 1.0), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory2), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(37))
                )), 
                new RunConveyorAction(1.0, 1.0), // shoot 3rd note
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory3), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(35))
                )), 
                new RunConveyorAction(1.0, 1.0), // shoot 4th note
                new ParallelAction(Arrays.asList(
                    new DrivePath(trajectory4), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(40))
                )), 
                new DrivePath(trajectory5),
                new RunConveyorAction(1.0, 1.0) // shoot 5th note
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "4 note close blue";
    }
}
