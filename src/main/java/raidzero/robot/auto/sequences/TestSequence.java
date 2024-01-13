package raidzero.robot.auto.sequences;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.submodules.Swerve;

public class TestSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();
    private PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test Path");
    private PathPlannerTrajectory testTrajectory;

    public TestSequence() {
        testTrajectory = testPath.getTrajectory(new ChassisSpeeds(), new Rotation2d());
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
