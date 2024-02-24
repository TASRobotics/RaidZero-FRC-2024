package raidzero.robot.auto.actions;

import edu.wpi.first.math.geometry.Rotation2d;

import raidzero.robot.submodules.AngleAdjuster;

public class AngleShooterAction implements Action {
    private static final AngleAdjuster mAngleAdjuster = AngleAdjuster.getInstance();

    private Rotation2d mAngle;

    public AngleShooterAction(Rotation2d angle) {
        mAngle = angle;
    }

    @Override
    public boolean isFinished() {
        return mAngleAdjuster.isFinished();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        mAngleAdjuster.setAngle(mAngle);
    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
