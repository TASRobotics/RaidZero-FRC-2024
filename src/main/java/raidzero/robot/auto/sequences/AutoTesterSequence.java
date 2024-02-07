package raidzero.robot.auto.sequences;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.auto.actions.Action;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.Res;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;


public class AutoTesterSquence extends AutoSequence {
    private PathPlannerPath path1 = PathPlannerPath.fromPathFile("Auto One.auto");

    public AutoTesterSequence() {
        path1 = path1.flipPath();
        Rotation2d test1 = new Rotation2d(Math.toRadians(0));
        trajectory1 = path1.getTrajectory(new ChassisSpeeds(), test1);
    }

    @Override
    public void sequence() {
        List<Action> lol = new ArrayList<>(); // Specify the type in the diamond operator
        lol.add(new Res());
        lol.add(new DrivePath(trajectory1));

        addAction(new SeriesAction(lol));
    }


    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "3 right flip";
    }
}

public class AutoSTesterSequence extends AutoSequence{

    }

    /**
     * Overridable method that is called at the end of the sequence.
     */
    public void onEnded() {
        System.out.println("[Auto] Auto sequence '" + getName() + "' ended!");
    }

    /**
     * Stops the autonomous sequence without calling {@link #onEnded()}
     */
    public void stop() {
        isSequenceRunning = false;
    }

    /**
     * Whether the sequence is currently running.
     * 
     * @return if the sequence is running
     */
    public boolean isRunning() {
        return isSequenceRunning;
    }

    /**
     * Adds an action to the action queue.
     * 
     * @param action action to add
     */
    public void addAction(Action action) {
        actions.add(action);
    }

    /**
     * Executes actions from the action queue.
     * 
     * Note: This should never be called by user code, only by
     * {@link AutoRunner}.
     * 
     * @param timestamp
     */
    public void onLoop(double timestamp) {
        // Don't run actions if the sequence is not running
        if (!isSequenceRunning) {
            return;
        }
        // Start the next action if we're done with the previous
        if (currentAction == null) {
            currentAction = actions.poll();
            currentAction.start();
        }
        currentAction.update();

        // Get ready to start the next action
        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;

            // End the sequence if we're done with all actions
            if (actions.isEmpty()) {
                onEnded();
                stop();
                return;
            }
        }
    }

    /**
     * Returns a user-friendly name for the autonomous sequence.
     * 
     * @return a user-friendly name
     */
    public String getName() {
        return "Nameless Sequence";
    }

    @Override
    public String toString() {
        return getName();
    }
}
