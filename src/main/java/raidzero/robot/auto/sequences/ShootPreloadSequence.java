package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import raidzero.robot.auto.actions.AngleShooterAction;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;

public class ShootPreloadSequence extends AutoSequence {

    public ShootPreloadSequence() {}

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new ParallelAction(Arrays.asList(
                new ShootAction(true), 
                new AngleShooterAction(Rotation2d.fromDegrees(48.69))
            )), 
            new RunConveyorAction(1.0, 1.0), 
            new ShootAction(false)
        )));
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "Shoot Preload Sequence";
    }
}
