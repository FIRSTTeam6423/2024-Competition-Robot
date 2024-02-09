// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Retention;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.CargoUtil;
import frc.robot.util.IronUtil;



public class IntakePivotTest extends Command {

  private CargoUtil cu;

  private TrapezoidProfile.State goalState;

  public IntakePivotTest(CargoUtil cu, TrapezoidProfile.State goalState) {
    this.cu = cu;
    this.goalState = goalState;
    addRequirements(cu);
  }

  @Override
  public void initialize() {
    cu.resetProfileTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cu.testIntakePivotToState(goalState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (IronUtil.deadzone(cu.getIntakeAngleRelativeToGround().getDegrees(), Constants.INTAKE_PIVOT_DEADBAND_DEGREES) == 0 ) {
      return true;
    } else {
      return false;
    }
  }
}
