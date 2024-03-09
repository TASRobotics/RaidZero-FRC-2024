package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.auto.AutoRunner;
import raidzero.robot.submodules.*;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.teleop.Teleop;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {
    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private static final Teleop mTeleop = Teleop.getInstance();
    private static final AngleAdjuster mAngleAdjuster = AngleAdjuster.getInstance();
    private static final Arm mArm = Arm.getInstance();
    private static final Climb mClimb = Climb.getInstance();
    private static final Intake mIntake = Intake.getInstance();
    // private static final Limelight mLimelight = Limelight.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Swerve mSwerve = Swerve.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();
    private static final Conveyor mConveyor = Conveyor.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private static final Vision mVision = Vision.getInstance();

    private AutoRunner autoRunner;

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Register all submodules here
        submoduleManager.setSubmodules(
            mWrist,
            mIntake,
            mConveyor,
            mSwerve,
            mVision,
            // mLimelight
            mArm, 
            // mClimb, 
            mShooter, 
            mAngleAdjuster, 
            mSuperstructure
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        autoRunner.readSendableSequence();
        autoRunner.start();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        // System.out.println("tx full: " + RobotController.getCANStatus().txFullCount);
        submoduleManager.onLoop(timestamp);
        autoRunner.onLoop(timestamp);

    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous
        autoRunner.stop();

        // Start the teleop handler
        mTeleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        mTeleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }
}
