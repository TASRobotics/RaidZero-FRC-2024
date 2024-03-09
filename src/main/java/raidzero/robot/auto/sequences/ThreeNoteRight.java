package raidzero.robot.auto.sequences;
import java.util.Arrays;

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
    private PathPlannerPath coopPreload = PathPlannerPath.fromPathFile("coop preload");
    private PathPlannerPath coopGet1 = PathPlannerPath.fromPathFile("coop get 1");
    private PathPlannerPath coopShoot1 = PathPlannerPath.fromPathFile("coop shoot 2");
    private PathPlannerPath coopGet2 = PathPlannerPath.fromPathFile("coop get 2");
    private PathPlannerPath coopShoot2 = PathPlannerPath.fromPathFile("coop shoot 2");

    public ThreeNoteRight() {
    }

    @Override
    public void sequence() {

        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(coopPreload.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(26.72))
                )), 
                new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(coopGet1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(coopShoot1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(5), 
                    new AngleShooterAction(Rotation2d.fromDegrees(26.72))
                )), 
               // new DrivePath(trajectory3), //go to shoot place
                new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new DrivePath(coopGet2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                        new DrivePath(coopShoot2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(5), 
                    new AngleShooterAction(Rotation2d.fromDegrees(26.72))
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
        return "3 co op blue";
    }
}
