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

public class RED_FiveNoteSpeaker extends AutoSequence {
    private PathPlannerPath redFiveNoteSpeaker0 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_0");
    private PathPlannerPath redFiveNoteSpeaker1 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_1");
    private PathPlannerPath redFiveNoteSpeaker2 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_2");
    private PathPlannerPath redFiveNoteSpeaker3 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_3");
    private PathPlannerPath redFiveNoteSpeakerGet4 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_Get4");
    private PathPlannerPath redFiveNoteSpeakerShoot4 = PathPlannerPath.fromPathFile("RED_FiveNoteSpeaker_Shoot4");

    public RED_FiveNoteSpeaker() {
    }

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new Res(), 
                new ParallelAction(Arrays.asList(
                    new DrivePath(redFiveNoteSpeaker0.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))), 
                    new ShootAction(true), 
                    new AngleShooterAction(Rotation2d.fromDegrees(39.09))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // Shoot 1st note (preload)
                        new DrivePath(redFiveNoteSpeaker1.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(33.49))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 2nd note
                        new DrivePath(redFiveNoteSpeaker2.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(28.83))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 3rd note
                        new DrivePath(redFiveNoteSpeaker3.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(35.5))
                )), 
                new ParallelAction(Arrays.asList(
                    new SeriesAction(Arrays.asList(
                        new RunConveyorAction(1.0, 0.5), // shoot 4th note
                        new DrivePath(redFiveNoteSpeakerGet4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180))))
                    )),
                    new AutomaticIntakeAction(), 
                    new AngleShooterAction(Rotation2d.fromDegrees(27.7))
                )), 
                new DrivePath(redFiveNoteSpeakerShoot4.getTrajectory(new ChassisSpeeds(), new Rotation2d(Math.toRadians(180)))),
                new RunConveyorAction(1.0, 0.5) // shoot 5th note
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "RED_FiveNoteSpeaker";
    }
}
