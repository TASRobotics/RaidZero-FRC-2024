package raidzero.robot.teleop;

import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.SwerveModule.PeriodicIO;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);
    private static GenericHID p3 = new GenericHID(2);

    private static Swerve mSwerve = Swerve.getInstance();

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

    private void p1Loop(XboxController p) {
        mSwerve.teleopDrive(p.getLeftY(), p.getLeftX(), p.getRightX(), false);
        // mSwerve.testModule(1, 0, p.getRightX()*.15);

        // mSwerve.getModules()[0].turnToAngleTest(p.getAButton());

        // SmartDashboard.putNumber("module angle", mSwerve.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("CANCoder Angle", mSwerve.getModules()[0].getNegatedAzimuthAngle());
        SmartDashboard.putNumber("Motor Angle", mSwerve.getModules()[0].getAzimuthMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Desired Azimuth Angle", mSwerve.getModules()[0].getPeriodicIO().desiredState.angle.getRadians()/(2*Math.PI));
    }

    private void p2Loop(XboxController p) {
       
    }
}
