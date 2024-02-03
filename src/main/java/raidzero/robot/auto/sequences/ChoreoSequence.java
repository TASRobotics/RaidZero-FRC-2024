package raidzero.robot.auto.sequences;

import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.TestChoreo;

public class ChoreoSequence extends AutoSequence {

    public ChoreoSequence() {

    }

    @Override
    public void sequence() {
addAction(
            new TestChoreo()
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "dsdsds Sequence";
    }
}
