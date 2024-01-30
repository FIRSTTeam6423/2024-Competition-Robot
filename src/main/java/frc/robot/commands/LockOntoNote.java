
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveUtil;

public class LockOntoNote extends CommandBase {
  /** Creates a new DriveRobot. */
  public PhotonCamera JohnCam = new PhotonCamera("johncam");
  private PIDController turnPID = new PIDController(60, 0, 0);
  private DriveUtil du;

  private Timer timer = new Timer();

  private double yaw;
  public double lockedRotation;
  private double angleTarget = 0;
  private Translation2d target = new Translation2d();
  public double deadzone(double input){
		if(Math.abs(input) >= Constants.XBOX_STICK_DEADZONE_WIDTH){
			return input;
		} else {
			return 0;
		}
	}
  
  public LockOntoNote(DriveUtil du) {
    this.du = du;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omega = 0;
    //if (timer.get() >= 2) {
      var result = JohnCam.getLatestResult();
      if (result.hasTargets() == false) {
        // omega = deadzone(RobotContainer.getDriverRightXboxX())
        //       * Math.toRadians(Constants.MAX_ANGULAR_SPEED) 
        //       * ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);
      } else {
        //omega = turnPID.calculate(du.getHeading2d().getRadians(), );
        List<PhotonTrackedTarget> targetList = result.getTargets();
        PhotonTrackedTarget target = targetList.get(0);
        
        for(int i = 1; i < targetList.size(); i++){
          if (Math.abs(targetList.get(i).getYaw()) < Math.abs(target.getYaw())) target = targetList.get(i);
        }
        omega = turnPID.calculate(-Units.degreesToRadians(target.getYaw()), 0);
        //angleTarget = du.getPose().getRotation().getDegrees() + result.getBestTarget().getYaw();
        //double yaw = result.getBestTarget().getYaw();
        //double yawRadians = Units.degreesToRadians(yaw);
        //double distance = PhotonUtils.calculateDistanceToTargetMeters(
        //  Constants.CAMERA_TO_ROBOT.getY(),
        //  0,
        //  0,
        //  Units.degreesToRadians(result.getBestTarget().getPitch())
        //);
        //SmartDashboard.putNumber("distance", distance);
        //target = du.getPose().getTranslation().minus(new Translation2d(distance * Math.cos(yawRadians), distance * Math.sin(yawRadians)));

        SmartDashboard.putNumber("X note position", yaw);
      }
      timer.reset();
      timer.start();
   // }
    //rotation = (yaw >= 50) ? 25 : yaw * yaw * 0.01;
    //SmartDashboard.putNumber("new rotation", yaw);
    // Use addRequirements() here to declare subsystem dependencies.

    //Pose2d robotPose = du.getPose();
    //angleTarget = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX());
    

    //SmartDashboard.putNumber("ANGLE TARGAHJ;GDSKHAODGKFBJ;D", Units.radiansToDegrees(angleTarget));

    //System.out.println(du.getPose().getRotation().getRadians() + " AND " + angleTarget);

    //omega = turnPID.calculate(du.getPose().getRotation().getRadians(), angleTarget);

    int xSign = (int)Math.signum(RobotContainer.getDriverLeftXboxY());
		double xSpeed = xSign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxY()), 2) 
						* Constants.MAX_LINEAR_SPEED 
						//* Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

		int ySign = (int)Math.signum(RobotContainer.getDriverLeftXboxX());
		double ySpeed = ySign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxX()), 2) 
						* Constants.MAX_LINEAR_SPEED 
						//* Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is
    du.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, du.getHeading2d()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!RobotContainer.getDriverLeftBumper()) return true;
    return false;
  }
}