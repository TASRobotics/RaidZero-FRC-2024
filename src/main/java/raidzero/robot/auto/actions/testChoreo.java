package raidzero.robot.auto.actions;

import com.choreo.lib.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

private ChoreoTrajectory traj = Choreo.getTrajectory("NewPath"); 
private Command toConvert;

public class testChoreo implements Action{

    public Action(){

    }

// 
Choreo.choreoSwerveCommand(
    traj, // 
    this::getPose // 
    new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
    new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
    new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // 
    (ChassisSpeeds speeds) -> // 
        this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), ...),
    () -> {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
    }, // 
    this, // 
);
}