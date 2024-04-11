// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commons.IronUtil;
import frc.robot.Drive.Drive;

public class OperateDrive extends Command {
  /** Creates a new OperateDrive. */
  private Drive du;

  public OperateDrive(Drive du) {
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
    double xInput = IronUtil.deadzone(RobotContainer.driver.getLeftY(), Constants.XBOX_STICK_DEADZONE_WIDTH);
    double yInput = IronUtil.deadzone(RobotContainer.driver.getLeftX(), Constants.XBOX_STICK_DEADZONE_WIDTH);
    double omegaInput = IronUtil.deadzone(RobotContainer.driver.getRightX(), Constants.XBOX_STICK_DEADZONE_WIDTH);

    SmartDashboard.putNumber("X INPUT; ", xInput);

    int xSign = (int)Math.signum(RobotContainer.driver.getLeftY()); //Must keep sign because we are squaring input
    double xSpeed = xSign * Math.pow(xInput, 2)  //NEED TO REVERSE DEPENDING ON ALLIANCE COLOR
            * Constants.MAX_LINEAR_SPEED 
            * ((RobotContainer.driver.getLeftTriggerAxis() > .5) ? .25 : 1); //reversed x and y so that up on controller is

    int ySign = (int)Math.signum(RobotContainer.driver.getLeftX()); //Must keep sign because we are squaring input
    double ySpeed = ySign * Math.pow(yInput, 2)  //NEED TO REVERSES DEPENDING ON ALLIANCE COLOR
            * Constants.MAX_LINEAR_SPEED 
            * ((RobotContainer.driver.getLeftTriggerAxis() > .5) ? .25 : 1); //reversed x and y so that up on controller is

    double omega =  omegaInput
            * Math.toRadians(Constants.MAX_ANGULAR_SPEED) 
            * ((RobotContainer.driver.getLeftTriggerAxis() > .5) ? .25 : 1);


    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, //reversed x and y so that up on controller is
                ySpeed, //forward from driver pov
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
