package raidzero.robot.teleop;

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
import raidzero.robot.utils.JoystickUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        p1Loop(p1);
        
        p2Loop(p2);
    }

    // int moduleNumber = 3;

    Rotation2d snapAngle = null;

    double desiredShooterSpeed = 90.0; 

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
        } else {
            snapAngle = null;
        }

        mSwerve.teleopDrive(
            JoystickUtils.applyDeadband(p.getLeftY()) * SwerveConstants.kMaxVelMPS, 
            JoystickUtils.applyDeadband(p.getLeftX()) * SwerveConstants.kMaxVelMPS, 
            JoystickUtils.applyDeadband(p.getRightX()) * SwerveConstants.kMaxVelMPS, 
            true, 
            snapAngle, 
            p.getXButton(),
            false
        );
    }

    private void p2Loop(XboxController p) {
       
    }
}
