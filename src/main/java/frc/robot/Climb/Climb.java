// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import frc.robot.Climb.ClimbConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private CANSparkMax leftClimber = new CANSparkMax(ClimbConstants.leftClimber, MotorType.kBrushless);
  private CANSparkMax rightClimber = new CANSparkMax(ClimbConstants.rightClimber, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftClimber.getEncoder();
  private RelativeEncoder rightEncoder = rightClimber.getEncoder();

  private PIDController leftController = new PIDController(ClimbConstants.leftClimberP, ClimbConstants.leftClimberI, ClimbConstants.leftClimberD);
  private PIDController rightController= new PIDController(ClimbConstants.rightClimberP, ClimbConstants.rightClimberI, ClimbConstants.rightClimberD);

  /** Creates a new Climb. */
  public Climb() {
    leftClimber.setInverted(true);
    rightClimber.setInverted(true);
  }
}
