package raidzero.robot.teleop;

import raidzero.robot.submodules.Swerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;


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
        mSwerve.teleopDrive(p.getLeftY(), p.getLeftY(), p.getRightX(), true);
    }

    private void p2Loop(XboxController p) {
       
    }
}
