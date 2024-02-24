package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.RunConveyorAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.ShootAction;

public class SingleNoteSpeaker extends AutoSequence {

    public SingleNoteSpeaker() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(Arrays.asList(
            new ShootAction(true), 
            new RunConveyorAction(1.0, 2), 
            new ShootAction(false)
        )));
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "Single Note Speaker";
    }
}
