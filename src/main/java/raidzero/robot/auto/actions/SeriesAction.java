package raidzero.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Action for running multiple Actions in series.
 */
public class SeriesAction implements Action {

    private Action currentAction;
    private final ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> actions) {
        remainingActions = new ArrayList<>(actions);
        currentAction = null;
    }

    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && currentAction == null;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (currentAction == null) {
            if (remainingActions.isEmpty()) {
                return;
            }
            currentAction = remainingActions.remove(0);
            currentAction.start();
            System.out.println("series action new action");
        }

        currentAction.update();

        if (currentAction.isFinished()) {
            System.out.println("series action isFinished");
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public void done() {
        System.out.println("series action done");
        if (currentAction != null) {
            currentAction.done();
        }
    }
}
