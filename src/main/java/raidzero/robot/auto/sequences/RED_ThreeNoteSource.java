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

public class RED_ThreeNoteSource extends AutoSequence {
    private PathPlannerPath redThreeNoteSource0 = PathPlannerPath.fromPathFile("RED_ThreeNoteSource_0");
    private PathPlannerPath redThreeNoteSourceGet1 = PathPlannerPath.fromPathFile("RED_ThreeNoteSource_Get1");
    private PathPlannerPath redThreeNoteSourceShoot1 = PathPlannerPath.fromPathFile("RED_ThreeNoteSource_Shoot1");
    private PathPlannerPath redThreeNoteSourceGet2 = PathPlannerPath.fromPathFile("RED_ThreeNoteSource_Get2");
    private PathPlannerPath redThreeNoteSourceShoot2 = PathPlannerPath.fromPathFile("RED_ThreeNoteSource_Shoot2");

    public RED_ThreeNoteSource() {
    }

    @Override
    public void sequence() {

        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(redThreeNoteSource0.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(27.75))
                )), 
                new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(redThreeNoteSourceGet1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                        new DrivePath(redThreeNoteSourceShoot1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new SeriesAction(Arrays.asList(
                        new WaitAction(0.9), // Delays a bit before dropping intake
                        new AutomaticIntakeAction(3)
                    )),
                    new AngleShooterAction(Rotation2d.fromDegrees(26))
                )), 
               // new DrivePath(trajectory3), //go to shoot place
                new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(redThreeNoteSourceGet2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                        new DrivePath(redThreeNoteSourceShoot2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new SeriesAction(Arrays.asList(
                        new WaitAction(0.75), // Delays a bit before dropping intake
                        new AutomaticIntakeAction(3)
                    )),
                    new AngleShooterAction(Rotation2d.fromDegrees(26))
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
        return "RED_ThreeNoteSource";
    }
}
