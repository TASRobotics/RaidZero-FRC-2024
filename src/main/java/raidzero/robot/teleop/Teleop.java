package raidzero.robot.teleop;

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
import raidzero.robot.submodules.SwerveModule.PeriodicIO;
import raidzero.robot.utils.JoystickUtils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);
    private static GenericHID p3 = new GenericHID(2);

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

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
    }

    public void onLoop() {
        p1Loop(p1);
        
        p2Loop(p2);
    }

    // int moduleNumber = 3;

    // Rotation2d snapAngle = null;

    double desiredShooterSpeed = 90.0; 

    private void p1Loop(XboxController p) {
        desiredShooterSpeed = SmartDashboard.getNumber("Desired Shooter Speed", desiredShooterSpeed);
        SmartDashboard.putNumber("Desired Shooter Speed", desiredShooterSpeed);

        SmartDashboard.putNumber("Shooter Angle", mAngleAdjuster.getAngle().getDegrees());
        SmartDashboard.putNumber("Arm Angle", mArm.getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Angle", mWrist.getAngle().getDegrees());

        double leftTrigger = p.getLeftTriggerAxis();
        double rightTrigger = p.getRightTriggerAxis();

        // if(p.getRightBumper()) {
        //     // mConveyor.setPercentSpeed(1.0);
        //     // mWrist.setPercentSpeed(0.3);
        //     // mAngleAdjuster.setAngle(Rotation2d.fromDegrees(30));
        //     mShooter.setVelocity(90);
        // } else {
        //     mShooter.setPercentSpeed(0.0);
        // }
        // if(p.getLeftBumper()) {
        //     // mConveyor.setPercentSpeed(-1.0);
        //     // mWrist.setPercentSpeed(-0.3);
        //     // mAngleAdjuster.setAngle(Rotation2d.fromDegrees(50));
        //     mConveyor.setPercentSpeed(1.0);
        // } else {
        //     // mConveyor.setPercentSpeed(0);
        //     // mWrist.setPercentSpeed(0.0);
        //     // mAngleAdjuster.setPercentSpeed(0);
        //     // mShooter.setPercentSpeed(0.0);
        //     mConveyor.setPercentSpeed(0.0);
        // }

        // if(p.getBButton()) {
        //     mIntake.setPercentSpeed(1.0, 1.0);
        // } else {
        //     mIntake.setPercentSpeed(0.0, 0.0);
        // }

        mArm.setPercentSpeed(leftTrigger - rightTrigger);

        // if(p.getYButton()) {
        //     // mArm.setAngle(SuperstructureConstants.kArmAmpAngle);
        //     mShooter.setVelocity(desiredShooterSpeed);
        // } else {
        //     // mArm.setPercentSpeed(0);
        //     mShooter.setPercentSpeed(0.0);
        // }

        // if(p.getXButton()) {
        //     // mWrist.setAngle(SuperstructureConstants.kWristStowAngle);
        //     // mArm.setAngle(SuperstructureConstants.kArmAmpAngle);
        //     // mIntake.setPercentSpeed(1.0, 1.0);
        //     mSuperstructure.stowState();
        // } else if(p.getAButton()) {
        //     // mWrist.setAngle(SuperstructureConstants.kWristIntakingAngle);
        //     // mArm.setAngle(SuperstructureConstants.kArmStowAngle);
        //     // mIntake.setPercentSpeed(-1.0, -1.0);
        //     mSuperstructure.ampState();
        // } else {
        //     // mWrist.setPercentSpeed(leftTrigger - rightTrigger);
        //     // mWrist.setPercentSpeed(0);
        //     // mArm.setPercentSpeed(0.0);
        //     // mIntake.setPercentSpeed(0.0, 0.0);
        // }

        // mAngleAdjuster.setPercentSpeed((leftTrigger - rightTrigger) * 0.5);
        // mArm.setPercentSpeed((leftTrigger - rightTrigger) * 0.5);

        

        // SmartDashboard.putNumber("Wrist angle", mWrist.getAngle().getDegrees());
        // SmartDashboard.putNumber("Current Shooter Speed", mShooter.getVelocity());
        // SmartDashboard.putNumber("Desired Shooter Speed", desiredShooterSpeed);
        // SmartDashboard.putNumber("Arm Angle", mArm.getAngle().getDegrees());

        // mShooter.setPercentSpeed(rightTrigger);


        mSwerve.teleopDrive(
            JoystickUtils.applyDeadband(p.getLeftY()) * SwerveConstants.kMaxVelMPS, 
            JoystickUtils.applyDeadband(p.getLeftX()) * SwerveConstants.kMaxVelMPS, 
            JoystickUtils.applyDeadband(p.getRightX()) * SwerveConstants.kMaxVelMPS, 
            true, 
            null, 
            p.getXButton(),
            false
        );

        // if(p.getXButton()) {
        //     // mSuperstructure.angleShooter();
        //     mSuperstructure.ampState();
        // } 
        if(p.getYButton()) {
            mWrist.setPercentSpeed(0.25);
        } else if(p.getAButton()) {
            mWrist.setPercentSpeed(-0.25);
        } else {
            mWrist.setPercentSpeed(0.0);
        }

        if(p.getLeftBumper()) {
            mClimb.setPercentSpeed(.25);
        } else if(p.getRightBumper()) {
            mClimb.setPercentSpeed(-.25);
        } else {
            mClimb.setPercentSpeed(0.0);
        }
        // else {
        //     mAngleAdjuster.setPercentSpeed(0.0);
        // }

        
        // if(p.getBButton()) {
        //     snapAngle = Rotation2d.fromDegrees(0);
        // } else {
        //     snapAngle = null;
        // }

        // mSwerve.teleopDrive(
        //     -JoystickUtils.applyDeadband(p.getLeftY()) * SwerveConstants.kRealisticMaxVelMPS, 
        //     -JoystickUtils.applyDeadband(p.getLeftX()) * SwerveConstants.kRealisticMaxVelMPS, 
        //     -JoystickUtils.applyDeadband(p.getRightX()) * SwerveConstants.kRealisticMaxVelMPS, 
        //     true, 
        //     snapAngle,
        //     p.getAButton() /*false*/
        // );

        // SmartDashboard.putNumber("Throttle Position 0", mSwerve.getModules()[0].getThrottleMotor().getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Position 1", mSwerve.getModules()[1].getThrottleMotor().getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Position 2", mSwerve.getModules()[2].getThrottleMotor().getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Position 3", mSwerve.getModules()[3].getThrottleMotor().getPosition().getValueAsDouble());

        // SmartDashboard.putNumber("Throttle Velocity 0", mSwerve.getModules()[0].getThrottleMotor().getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Velocity 1", mSwerve.getModules()[1].getThrottleMotor().getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Velocity 2", mSwerve.getModules()[2].getThrottleMotor().getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Throttle Velocity 3", mSwerve.getModules()[3].getThrottleMotor().getVelocity().getValueAsDouble());

        // SmartDashboard.putNumber("CANCoder Angle", mSwerve.getModules()[moduleNumber].getAzimuthEncoder().getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Motor Angle", mSwerve.getModules()[moduleNumber].getAzimuthMotor().getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Desired Azimuth Angle", mSwerve.getModules()[moduleNumber].getPeriodicIO().desiredState.angle.getRadians()/(2*Math.PI));
        // SmartDashboard.putNumber("Desired Azimuth Angle 2", mSwerve.getModules()[0].getPeriodicIO().desiredState.angle.getRadians()/(2*Math.PI));
    }

    private void p2Loop(XboxController p) {
       
    }
}
