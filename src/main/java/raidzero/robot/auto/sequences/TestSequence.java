package raidzero.robot.auto.sequences;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.DrivePath;

public class TestSequence extends AutoSequence {
    private PathPlannerPath testPath = PathPlannerPath.fromPathFile("line");
    private PathPlannerTrajectory testTrajectory;

    public TestSequence() {
        Rotation2d test = new Rotation2d(Math.toRadians(0));
        testTrajectory = testPath.getTrajectory(new ChassisSpeeds(), test);
    }

    @Override
    public void sequence() {
        addAction(
            new DrivePath(testTrajectory)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}
