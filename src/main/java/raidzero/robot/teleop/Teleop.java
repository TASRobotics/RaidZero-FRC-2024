package raidzero.robot.teleop;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.SwerveModule.PeriodicIO;
import raidzero.robot.utils.JoystickUtils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private static final Shooter shooter = Shooter.getInstance();

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

    int moduleNumber = 3;

    Rotation2d snapAngle = null;

    private void p1Loop(XboxController p) {
        // mSwerve.teleopDrive(
        //     JoystickUtils.applyDeadband(p.getLeftY()), 
        //     JoystickUtils.applyDeadband(p.getLeftX()), 
        //     JoystickUtils.applyDeadband(p.getRightX()), 
        //     true
        // );
        if(p.getBButton()) {
            snapAngle = Rotation2d.fromDegrees(0);
        } else {
            snapAngle = null;
        }

        mSwerve.teleopDrive(
            -JoystickUtils.applyDeadband(p.getLeftY()) * SwerveConstants.kRealisticMaxVelMPS, 
            -JoystickUtils.applyDeadband(p.getLeftX()) * SwerveConstants.kRealisticMaxVelMPS, 
            -JoystickUtils.applyDeadband(p.getRightX()) * SwerveConstants.kRealisticMaxVelMPS, 
            true, 
            snapAngle,
            p.getAButton() /*false*/
        );

        // mIntake.setPercentSpeed(p.getLeftTriggerAxis() - p.getRightTriggerAxis());
        // if(p.getAButton()) {
        //     mSwerve.setClosedLoopSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0), true);
        // }
        // if (p.getAButton()) {
        //     intake.setPercentSpeed(0.45);
        // }
        // else if(p.getYButton()){
        //     intake.setPercentSpeed(-0.45);
        // }
        // if(p.getXButton()){
        //     shooter.shoot(1, false);
        // }
        // else{
        //     shooter.shoot(0, false);
        // }

        SmartDashboard.putNumber("Throttle Position 0", mSwerve.getModules()[0].getThrottleMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Position 1", mSwerve.getModules()[1].getThrottleMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Position 2", mSwerve.getModules()[2].getThrottleMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Position 3", mSwerve.getModules()[3].getThrottleMotor().getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Throttle Velocity 0", mSwerve.getModules()[0].getThrottleMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Velocity 1", mSwerve.getModules()[1].getThrottleMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Velocity 2", mSwerve.getModules()[2].getThrottleMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Throttle Velocity 3", mSwerve.getModules()[3].getThrottleMotor().getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("CANCoder Angle", mSwerve.getModules()[moduleNumber].getAzimuthEncoder().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Angle", mSwerve.getModules()[moduleNumber].getAzimuthMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Desired Azimuth Angle", mSwerve.getModules()[moduleNumber].getPeriodicIO().desiredState.angle.getRadians()/(2*Math.PI));
        SmartDashboard.putNumber("Desired Azimuth Angle 2", mSwerve.getModules()[0].getPeriodicIO().desiredState.angle.getRadians()/(2*Math.PI));
    }

    private void p2Loop(XboxController p) {
       
    }
}
