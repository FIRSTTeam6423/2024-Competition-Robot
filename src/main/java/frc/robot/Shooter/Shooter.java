// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDSubsystem;
import frc.robot.commons.IronUtil;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR, MotorType.kBrushless); 
  private CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private PIDController leftController = new PIDController(ShooterConstants.LEFT_ROLLER_P, ShooterConstants.LEFT_ROLLER_I, ShooterConstants.LEFT_ROLLER_D);
  private PIDController rightController = new PIDController(ShooterConstants.RIGHT_ROLLER_P, ShooterConstants.RIGHT_ROLLER_I, ShooterConstants.RIGHT_ROLLER_D);

  private double goalLeft = 0;
  private double goalRight = 0;

  private boolean enabled = false;

  public Shooter() {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
  }

  public double getMeasurementLeft() {
    SmartDashboard.putNumber("shooter rpm", leftEncoder.getVelocity());
    return leftEncoder.getVelocity(); //rpm
  } 

  public double getMeasurementRight() {
    return rightEncoder.getVelocity();
  }

  public void useOutputLeft(double output) {
    leftMotor.set(output);
  }

  public void useOutputRight(double output) {
    rightMotor.set(output);
  }

  public boolean atRPM() {
    return IronUtil.inRange(getMeasurementLeft(), ShooterConstants.SHOOT_RPM, 400);
  }

  public Command spinup() {
    return this.run(()->{
      useOutputLeft(leftController.calculate(getMeasurementLeft(), ShooterConstants.SHOOT_RPM));
      useOutputRight(rightController.calculate(getMeasurementRight(), ShooterConstants.SHOOT_RPM));
    });
  }

  public Command startSpinup() {
    return this.runOnce(()->{
      enabled = true;
      goalLeft = ShooterConstants.SHOOT_RPM;
      goalRight = ShooterConstants.SHOOT_RPM;
    });
  }
  
  public Command stopRollers() {
    return this.runOnce(()->{
      enabled = false;
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  public Command feed(){
    return this.run(()->{
      leftMotor.set(ShooterConstants.AMP_MECH_FEED_SPEED);
      rightMotor.set(ShooterConstants.AMP_MECH_FEED_SPEED);
    });
  }

public Command feedSlow() {
  return this.run(() ->{
    leftMotor.set(ShooterConstants.AMP_MECH_FEED_SPEED);
    rightMotor.set(ShooterConstants.AMP_MECH_FEED_SPEED);
  });

  
}


  public Command suckIn(){
    return this.run(() ->{
      rightMotor.set(ShooterConstants.AMP_MECH_SUCK_IN_SPEED);
      leftMotor.set(ShooterConstants.AMP_MECH_SUCK_IN_SPEED);
    });
  }
  @Override
  public void periodic() {
    if(enabled) {
      useOutputLeft(leftController.calculate(getMeasurementLeft(), goalLeft));
      useOutputRight(rightController.calculate(getMeasurementRight(), goalRight));
    }
  }
  
}
