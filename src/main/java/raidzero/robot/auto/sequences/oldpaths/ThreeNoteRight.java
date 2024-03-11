package raidzero.robot.auto.sequences;
import java.util.Arrays;

import com.pathplanner.lib.path.PathPlannerPath;

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
import raidzero.robot.auto.actions.WaitAction;

public class ThreeNoteRight extends AutoSequence {
    private PathPlannerPath preload = PathPlannerPath.fromPathFile("SourceSide3_Preload_Blue");
    private PathPlannerPath get1 = PathPlannerPath.fromPathFile("SourceSide3_Get1_Blue");
    private PathPlannerPath shoot1 = PathPlannerPath.fromPathFile("SourceSide3_Shoot1_Blue");
    private PathPlannerPath get2 = PathPlannerPath.fromPathFile("SourceSide3_Get2_Blue");
    private PathPlannerPath shoot2 = PathPlannerPath.fromPathFile("SourceSide3_Shoot2_Blue");

    public ThreeNoteRight() {
    }

    @Override
    public void sequence() {

        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(preload.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(26.75))
                )), 
                new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(get1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(shoot1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new SeriesAction(Arrays.asList(
                        new WaitAction(0.9), // Delays a bit before dropping intake
                        new AutomaticIntakeAction(5)
                    )),
                    new AngleShooterAction(Rotation2d.fromDegrees(25))
                )), 
               // new DrivePath(trajectory3), //go to shoot place
                new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(get2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(shoot2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new SeriesAction(Arrays.asList(
                        new WaitAction(0.75), // Delays a bit before dropping intake
                        new AutomaticIntakeAction(5)
                    )),
                    new AngleShooterAction(Rotation2d.fromDegrees(25))
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
        return "3 Note Source Side";
    }
}
