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

public class BLUE_ThreeNoteSource extends AutoSequence {
    private PathPlannerPath blueThreeNoteSource0 = PathPlannerPath.fromPathFile("BLUE_ThreeNoteSource_0");
    private PathPlannerPath blueThreeNoteSourceGet1 = PathPlannerPath.fromPathFile("BLUE_ThreeNoteSource_Get1");
    private PathPlannerPath blueThreeNoteSourceShoot1 = PathPlannerPath.fromPathFile("BLUE_ThreeNoteSource_Shoot1");
    private PathPlannerPath blueThreeNoteSourceGet2 = PathPlannerPath.fromPathFile("BLUE_ThreeNoteSource_Get2");
    private PathPlannerPath blueThreeNoteSourceShoot2 = PathPlannerPath.fromPathFile("BLUE_ThreeNoteSource_Shoot2");

    public BLUE_ThreeNoteSource() {
    }

    @Override
    public void sequence() {

        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(blueThreeNoteSource0.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(26.75))
                )), 
                new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(blueThreeNoteSourceGet1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(blueThreeNoteSourceShoot1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
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
                        new DrivePath(blueThreeNoteSourceGet2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(blueThreeNoteSourceShoot2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
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
        return "BLUE_ThreeNoteSource";
    }
}
