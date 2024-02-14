// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Retention;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CargoUtil;
import frc.robot.util.CargoState;
import frc.robot.util.IronUtil;



public class IntakePivotTest extends Command {

  private CargoUtil cu;

  private TrapezoidProfile.State goalState;

  private Timer stateSwitchTimer = new Timer();

  private Boolean done = false;

  public IntakePivotTest(CargoUtil cu) {
    this.cu = cu;
    addRequirements(cu);
  }

  @Override
  public void initialize() {
    cu.resetProfileTimer();
    cu.setState(CargoState.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cu.operateCargoMachine();
    if(RobotContainer.getDriverIntakeInput() && cu.getState() == CargoState.IDLE) {
      cu.setState(CargoState.INTAKING);
    }
    if(!RobotContainer.getDriverIntakeInput() && cu.getState() == CargoState.INTAKING) {
      //cu.setState(CargoState.IDLE);
    }
    if(RobotContainer.getOperatorSpinupInput() && cu.getState() == CargoState.STOW) {
      cu.setState(CargoState.SPINUP);
    }
    if(!RobotContainer.getOperatorSpinupInput() && cu.getState() == CargoState.SPINUP) {
      cu.setState(CargoState.STOW);
    }
    if(RobotContainer.getDriverFireButton() && cu.getState() == CargoState.SPINUP) {
      cu.setState(CargoState.SHOOT);
    }
    // if (cu.intakeAtSetpoint() && done == false) {
    //   stateSwitchTimer.restart();
    //   done = true;
    // }
    // if (stateSwitchTimer.get() > 2) {
    //   cu.setState(CargoState.IDLE);
    // }
    //cu.testIntakePivotToState(goalState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (cu.getAmpMechAngleRelativeToGround().getDegrees() < 0.1 && cu.getAmpMechAngleRelativeToGround().getDegrees() > -.1) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
