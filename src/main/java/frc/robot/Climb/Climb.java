// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax leftClimb, rightClimb;

  private RelativeEncoder leftClimbEncoder, rightClimbEncoder;

  public Climb() {
    //leftClimb = new CANSparkMax(ClimbConstants.CLIMB_LEFT_MOTOR, MotorType.kBrushless);
    
    rightClimb = new CANSparkMax(ClimbConstants.CLIMB_RIGHT_MOTOR, MotorType.kBrushless);
    
    leftClimb = new CANSparkMax(ClimbConstants.CLIMB_LEFT_MOTOR, MotorType.kBrushless);


    leftClimbEncoder = leftClimb.getEncoder();
    rightClimbEncoder = rightClimb.getEncoder();
    
    leftClimb.setInverted(true);
    rightClimb.setInverted(false);

    // ! Might break stuff idk lol
    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
  }

  public Command StopClimb() {
    return this.runOnce(() -> {
        leftClimb.stopMotor(); //im not sure this is gonna work beccause its gonna move up slightly 
        rightClimb.stopMotor(); //we could send volts every like 10 seconds to keep it down
      }
    );
  }

  public Command OperateClimb() {
    return this.run(()-> {
        double leftInput = RobotContainer.getOperatorLeftXboxY();
        double rightInput = RobotContainer.getOperatorRightXboxY();

        if (leftInput > 0) {
          leftClimb.set(leftInput * ClimbConstants.MAX_EXTEND_VOLTAGE);
        } 
        else if (leftInput < 0) {
          leftClimb.set(leftInput * ClimbConstants.MAX_RETRACT_VOLTAGE);
        }
        
        if (rightInput > 0) {
          rightClimb.set(rightInput * ClimbConstants.MAX_EXTEND_VOLTAGE);
        } 
        else if (rightInput < 0) {
          rightClimb.set(rightInput * ClimbConstants.MAX_RETRACT_VOLTAGE);
        }

        else {
          leftClimb.stopMotor();
          rightClimb.stopMotor();
        }
      }
    );
  }
}
