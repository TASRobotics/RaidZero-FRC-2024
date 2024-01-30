package raidzero.robot.auto.sequences;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.DrivePath;

public class ThreeNote extends AutoSequence {
    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("first note");
    private PathPlannerTrajectory trajectory1;

    public TestSequence() {
        Rotation2d test1 = new Rotation2d(Math.toRadians(-90));
        trajectory1 = testPath.getTrajectory(new ChassisSpeeds(), test);
    }

    @Override
    public void sequence() {
        addAction(
            new DrivePath(trajectory1)
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 note (with pause)";
    }
}
