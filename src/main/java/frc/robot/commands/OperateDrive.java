// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// TODO remove this it's deprecated
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commons.Utility;
import frc.robot.subsystems.DriveUtil;

public class OperateDrive extends Command {
  /** Creates a new OperateDrive. */
  private DriveUtil du;

  public OperateDrive(DriveUtil du, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.du = du;
	
    addRequirements(this.du);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	double xInput = Utility.deadzone(RobotContainer.getDriverLeftXboxX(), Constants.XBOX_STICK_DEADZONE_WIDTH);
	double yInput = Utility.deadzone(RobotContainer.getDriverLeftXboxY(), Constants.XBOX_STICK_DEADZONE_WIDTH);
	double omegaInput = Utility.deadzone(RobotContainer.getDriverRightXboxX(), Constants.XBOX_STICK_DEADZONE_WIDTH);

	double xSpeed = Utility.squareInputKeepSign(xInput)  //NEED TO REVERSE DEPENDING ON ALLIANCE COLOR
					* Constants.MAX_LINEAR_SPEED 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

	double ySpeed = Utility.squareInputKeepSign(yInput)  //NEED TO REVERSES DEPENDING ON ALLIANCE COLOR
					* Constants.MAX_LINEAR_SPEED 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); //reversed x and y so that up on controller is

	double omega =  omegaInput
					* Math.toRadians(Constants.MAX_ANGULAR_SPEED) 
					* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);

	ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
							ySpeed, //reversed x and y so that up on controller is
							xSpeed, //forward from driver pov
							omega, 
							du.getHeading2d());

	du.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}