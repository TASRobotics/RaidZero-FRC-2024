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

public class BLUE_FiveNoteSpeaker extends AutoSequence {
    private PathPlannerPath blueFiveNoteSpeaker0 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_0");
    private PathPlannerPath blueFiveNoteSpeaker1 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_1");
    private PathPlannerPath blueFiveNoteSpeaker2 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_2");
    private PathPlannerPath blueFiveNoteSpeaker3 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_3");
    private PathPlannerPath blueFiveNoteSpeakerGet4 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_Get4");
    private PathPlannerPath blueFiveNoteSpeakerShoot4 = PathPlannerPath.fromPathFile("BLUE_FiveNoteSpeaker_Shoot4");

    public BLUE_FiveNoteSpeaker() {
    }

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(blueFiveNoteSpeaker0.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(39.09))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                        new DrivePath(blueFiveNoteSpeaker1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(33.49))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                        new DrivePath(blueFiveNoteSpeaker2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(28.83))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 3rd note
                        new DrivePath(blueFiveNoteSpeaker3.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(35.5))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 4th note
                        new DrivePath(blueFiveNoteSpeakerGet4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(27.7))
                )), 
                new DrivePath(blueFiveNoteSpeakerShoot4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(0)))),
                new RunConveyorAction(1.0, 0.5) // shoot 5th note
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "BLUE_FiveNoteSpeaker";
    }
}
