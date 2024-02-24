package raidzero.robot.auto.actions;

import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Wrist;

public class AutomaticIntakeAction implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();

    private double mSpeed;

    public AutomaticIntakeAction(double speed) {
        mSpeed = speed;
    }

    @Override
    public boolean isFinished() {
        return mIntake.ringPresent();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
        mIntake.setPercentSpeed(mSpeed, mSpeed);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        mIntake.setPercentSpeed(0.0, 0.0);
        mWrist.setAngle(SuperstructureConstants.kArmStowAngle);
    }
}
