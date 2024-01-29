package raidzero.robot.submodules;

public class Superstructure extends Submodule {

    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {}

    //private static final Intake mIntake = Intake.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Arm mArm = Arm.getInstance();

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {}

    @Override
    public void stop() {}

    @Override
    public void zero() {}
}