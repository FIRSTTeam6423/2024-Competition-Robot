// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private CANSparkMax leftClimber, rightClimber;
  
  private RelativeEncoder leftEncoder, rightEncoder;

  private PIDController leftController, rightController;
  
  /** Creates a new Climb. */
  public Climb() {
    leftClimber = new CANSparkMax(ClimbConstants.leftClimber, MotorType.kBrushless);
    rightClimber = new CANSparkMax(ClimbConstants.rightClimber, MotorType.kBrushless);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
    
    leftClimber.setInverted(true);
    
    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    leftController = new PIDController(ClimbConstants.leftClimberP, ClimbConstants.leftClimberI, ClimbConstants.leftClimberD);
    rightController = new PIDController(ClimbConstants.rightClimberP, ClimbConstants.rightClimberI, ClimbConstants.rightClimberD);
    
    
  }



  public void resetEncoder() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Command setLeftClimberState() {
    return this.runOnce(() -> {
        return;
      }
    );
  }

  public Command setRightClimberState() {
    return this.runOnce(() -> {
        return;
      }
    );
  }
}