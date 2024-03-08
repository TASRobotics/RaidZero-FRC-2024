package raidzero.robot.auto.sequences;
import java.util.Arrays;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.AngleShooterAction;
import raidzero.robot.auto.actions.AutomaticIntakeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;

public class ThreeNoteRed extends AutoSequence {
    private PathPlannerPath threeNotePreload = PathPlannerPath.fromPathFile("three note red preload");
    private PathPlannerPath threeNote1 = PathPlannerPath.fromPathFile("three note red 1");
    private PathPlannerPath threeNote2 = PathPlannerPath.fromPathFile("three note red 2");
    private PathPlannerPath threeNote3 = PathPlannerPath.fromPathFile("three note red 3");
    private PathPlannerPath threeNoteGet4 = PathPlannerPath.fromPathFile("three note red get 4");
    private PathPlannerPath threeNoteShoot4 = PathPlannerPath.fromPathFile("three note red shoot 4");

    public ThreeNoteRed() {
    }

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(threeNotePreload.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(45))
                )), 
                new RunConveyorAction(1.0, 1.0), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new DrivePath(threeNote1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(37))
                )), 
                new RunConveyorAction(1.0, 1.0), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new DrivePath(threeNote2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(37))
                )), 
                new RunConveyorAction(1.0, 1.0), // shoot 3rd note
                new ParallelAction(Arrays.asList(
                    new DrivePath(threeNote3.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(35))
                )), 
                new RunConveyorAction(1.0, 1.0), // shoot 4th note
                new ParallelAction(Arrays.asList(
                    new DrivePath(threeNoteGet4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(40))
                )), 
                new DrivePath(threeNoteShoot4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))),
                new RunConveyorAction(1.0, 1.0) // shoot 5th note
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {

        return "4 note close red";

    }
}
