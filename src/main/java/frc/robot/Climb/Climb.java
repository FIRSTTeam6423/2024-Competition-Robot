// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Climb.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkBase leftClimb, rightClimb;

  //private RelativeEncoder leftClimbEncoder, rightClimbEncoder;

  public Climb() {
    leftClimb.setInverted(true);
    rightClimb.setInverted(false);
  }
  
  public Command StopClimb() {
    return this.runOnce(() -> {
        leftClimb.stopMotor();
        rightClimb.stopMotor();
      }
    );
  }

  public Command OperateClimb() {
    return this.run(()-> {
        double leftInput = RobotContainer.getDriverLeftXboxY();
        double rightInput = RobotContainer.getDriverRightXboxY();

        if (leftInput > 0) {
          leftClimb.set(leftInput * ClimbConstants.MAX_EXTEND_VOLTAGE);
        } else if (leftInput < 0) {
          leftClimb.set(leftInput * ClimbConstants.MAX_RETRACT_VOLTAGE);
        }

        if (rightInput > 0) {
          rightClimb.set(rightInput * ClimbConstants.MAX_EXTEND_VOLTAGE);
        } else if (rightInput < 0) {
          rightClimb.set(rightInput * ClimbConstants.MAX_RETRACT_VOLTAGE);
        }
      }
    );
  }
}
