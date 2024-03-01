package raidzero.robot.teleop;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SuperstructureConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.AngleAdjuster;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Conveyor;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.submodules.Superstructure.SuperstructureState;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.LimelightHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    // private static XboxController p2 = new XboxController(1);
    private static GenericHID p2 = new GenericHID(1);

    private static Swerve mSwerve = Swerve.getInstance();

    // private static final Intake mIntake = Intake.getInstance();
    private static final Shooter mShooter = Shooter.getInstance();
    private static final Intake mIntake = Intake.getInstance();
    private static final Conveyor mConveyor = Conveyor.getInstance();
    private static final Wrist mWrist = Wrist.getInstance();
    private static final AngleAdjuster mAngleAdjuster = AngleAdjuster.getInstance();
    private static final Climb mClimb = Climb.getInstance();
    private static final Arm mArm = Arm.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private boolean mBlue = false;
    private int mReverse = 1;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
        mBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        mReverse = mBlue ? 1 : -1;
    }

    public void onLoop() {
        SmartDashboard.putBoolean("Blue Alliance?", mBlue);
        // mWrist.setPercentSpeed(p1.getLeftTriggerAxis() - p1.getRightTriggerAxis());
        // SmartDashboard.putNumber("Wrist angle", mWrist.getAngle().getDegrees());
        p1Loop(p1);
        
        p2Loop(p2);
        // mAngleAdjuster.setPercentSpeed(p1.getLeftTriggerAxis() - p1.getRightTriggerAxis());
        // if(p1.getLeftBumper()) {
        //     mAngleAdjuster.setAngle(Rotation2d.fromDegrees(30));
        //     // mAngleAdjuster.setPercentSpeed(0.2);
        // } else if(p1.getRightBumper()) {
        //     mAngleAdjuster.setAngle(Rotation2d.fromDegrees(45));
        //     // mAngleAdjuster.setPercentSpeed(-0.2);
        // } else {
        //     mAngleAdjuster.setPercentSpeed(0.0);
        // }
    }

    // int moduleNumber = 3;

    Rotation2d snapAngle = null;
    boolean autoAim = false;

    double desiredShooterSpeed = 90.0; 
    boolean intaking = false;

    boolean rightTriggerPressed = false;

    private void p1Loop(XboxController p) {

        desiredShooterSpeed = SmartDashboard.getNumber("Desired Shooter Speed", desiredShooterSpeed);
        SmartDashboard.putNumber("Desired Shooter Speed", desiredShooterSpeed);

        SmartDashboard.putNumber("Shooter Angle", mAngleAdjuster.getAngle().getDegrees());
        SmartDashboard.putNumber("Arm Angle", mArm.getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Angle", mWrist.getAngle().getDegrees());

        double leftTrigger = p.getLeftTriggerAxis();
        double rightTrigger = p.getRightTriggerAxis();

        if(p.getAButton()) {
            if(DriverStation.getAlliance().get() == Alliance.Blue) {
                snapAngle = Rotation2d.fromDegrees(0);
            } else {
                snapAngle = Rotation2d.fromDegrees(180);
            }
        } else if(isAttemptingToTurn()) {
            snapAngle = null;
        }

        if(p.getBButton()) {
            autoAim = true;
        } else if(isAttemptingToTurn()) {
            autoAim = false;
        }

        // if(LimelightHelpers.getTV(null))

        mSwerve.teleopDrive(
            -JoystickUtils.applyDeadband(p.getLeftY()) * SwerveConstants.kMaxVelMPS * 0.5, 
            -JoystickUtils.applyDeadband(p.getLeftX()) * SwerveConstants.kMaxVelMPS * 0.5, 
            -JoystickUtils.applyDeadband(p.getRightX()) * SwerveConstants.kMaxVelMPS * 0.5, 
            true, 
            null, 
            autoAim,
            p.getYButton()
        );

        if(autoAim) {
            mSuperstructure.angleShooter();
        }

        // if(p.getLeftBumper()) {
        //     mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
        //     // mIntake.setPercentSpeed(1.0, 1.0);
        // } else if(p.getLeftTriggerAxis() > 0.5) {
        //     mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
        // } 

        // if(p.getRightBumper()) {
        //     mSuperstructure.intakeChoreographed(true);
        // } else if(p.getRightTriggerAxis() > 0.5) {
        //     mIntake.setPercentSpeed(-1.0, -1.0);
        // } else {
        //     mIntake.setPercentSpeed(0.0, 0.0);
        //     mSuperstructure.intakeChoreographed(false);
        // }

        mSuperstructure.intakeChoreographed(p.getRightBumper(), p.getRightBumperReleased(), false);
        mSuperstructure.intakeChoreographed(p.getLeftBumper(), p.getLeftBumperReleased(), true);

        // if(mIntake.ringPresent()) {
        //     p.setRumble(RumbleType.kBothRumble, 1.0);
        // } else {
        //     p.setRumble(RumbleType.kBothRumble, 0.0);
        // }



        // else if(p.getRightBumperReleased() || isRightTriggerReleased()) {
        //     mIntake.setPercentSpeed(0.0, 0.0);
        // }



        // prevRightTrigger = p.getRightTriggerAxis() > 0.5;
        // if(p.getLeftBumper()) {
        //     mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
        // } else if(p.getRightBumper()) {
        //     mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
        // }

        // mIntake.setPercentSpeed(rightTrigger - leftTrigger, rightTrigger - leftTrigger);
    }

    private void p2Loop(GenericHID p) {
        // manual conveyor control
        if(p.getRawButton(6)) {
            mConveyor.setPercentSpeed(1.0);
        } else if(p.getRawButton(7)) {
            mConveyor.setPercentSpeed(-1.0);
        } else if(p.getRawButtonReleased(6) || p.getRawButtonReleased(7)) {
            mConveyor.setPercentSpeed(0.0);
        }

        // Amp
        if(p.getRawButton(10)) {
            mSuperstructure.ampState();
        }
        if(p.getRawButton(9)) {
            mSuperstructure.stowState();
        }
        if(p.getRawButton(12)) {
            mSuperstructure.trapState();
        }

        // Score
        if(p.getRawButton(11)) {
            if(mSuperstructure.getState() == SuperstructureState.AMP) {
                mIntake.setPercentSpeed(-1.0, -1.0);
            } else {
                // mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
                // if(mWrist.isSettled()) {
                //     mIntake.setPercentSpeed(1.0, 1.0);
                //     mConveyor.setPercentSpeed(1.0);
                // }
                mConveyor.setPercentSpeed(1.0);
            }
        } else if(p.getRawButtonReleased(11)) {
            if(mSuperstructure.getState() == SuperstructureState.AMP) {
                mIntake.setPercentSpeed(0.0, 0.0);
            } else if(!intaking) {
                // mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
                // if(mWrist.isSettled()) {
                //     mIntake.setPercentSpeed(0.0, 0.0);
                //     mConveyor.setPercentSpeed(0.0);
                // }
                mConveyor.setPercentSpeed(0.0);
            }
        }

        // Shooter
        if(p.getRawButton(13)) { 
            mShooter.setVelocity(90);
        } else if(p.getRawButton(1)) {
            mShooter.setPercentSpeed(0.0);
        }

        // wrist zeroing
        // if(p.getRawButton(3)) { // if button pressed
        //     mWrist.setPercentSpeed(-0.1);
        // } 

        if(p.getRawButton(3)) {
            mClimb.setPercentSpeed(0.2);
        } else if(p.getRawButton(2)) {
            mClimb.setPercentSpeed(-0.2);
        } else {
            mClimb.setPercentSpeed(0.0);
        }
    }

    private boolean isRightTriggerReleased() {
        boolean pressed = p1.getRightTriggerAxis() > 0.5;
        if(rightTriggerPressed && !pressed) {
            rightTriggerPressed = false;
            return true;
        } else if(pressed) {
            rightTriggerPressed = true; 
        }
        return false;
    }

    private boolean isAttemptingToTurn() {
        return Math.abs(p1.getRightX()) > Constants.kJoystickDeadband;
    }
}
