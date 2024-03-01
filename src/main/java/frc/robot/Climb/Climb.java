// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climb;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commons.IronUtil;

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

    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
  }

  public Command StopClimb() {
    return this.runOnce(() -> {
      leftClimb.stopMotor(); //im not sure this is gonna work beccause its gonna move up slightly 
      rightClimb.stopMotor(); //we could send volts every like 10 seconds to keep it down
    });
  }

  public Command setVoltage(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    return this.run(()->{
      double leftInput = leftSupplier.get();
      double rightInput = rightSupplier.get();
      double lmax = (leftInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE);
      double rmax = (rightInput < 0 ? ClimbConstants.MAX_RETRACT_VOLTAGE : ClimbConstants.MAX_EXTEND_VOLTAGE); 
      System.out.println(leftInput * lmax);
      leftClimb.setVoltage(leftInput * lmax);
      rightClimb.setVoltage(rightInput * rmax);
    });
  }

  public double getAverageCurrent() {
    return (leftClimb.getOutputCurrent() + rightClimb.getOutputCurrent()) / 2;
  }

  public boolean atCurrentLimit() {
    return getAverageCurrent() > ClimbConstants.MAX_CURRENT_AMPS;
  }
}
