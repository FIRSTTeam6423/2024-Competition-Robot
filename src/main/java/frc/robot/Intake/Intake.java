// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends ProfiledPIDSubsystem {

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.INTAKE_PIVOT_ENCODER);

  private CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
      IntakeConstants.INTAKE_PIVOT_kS, 
      IntakeConstants.INTAKE_PIVOT_kG,
      IntakeConstants.INTAKE_PIVOT_kV,
      IntakeConstants.INTAKE_PIVOT_kA
    );

  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(7), // TODO gotta fix outputs
    new DigitalInput(8),
    new DigitalInput(9),
  };
  
  /** Creates a new Intake. */
  public Intake() {
    super(
      new ProfiledPIDController(
            IntakeConstants.INTAKE_PIVOT_P,
            IntakeConstants.INTAKE_PIVOT_I,
            IntakeConstants.INTAKE_PIVOT_D,
            new TrapezoidProfile.Constraints(
                IntakeConstants.INTAKE_MAX_VELOCITY_DEG_PER_SEC,
                IntakeConstants.INTAKE_MAX_ACCELERATION_DEG_PER_SEC_SQUARED
            )
      ),
      0
    );
    pivotMotor.setInverted(true);
  }

  private Rotation2d getAngleRelativeToGround() {
     return Rotation2d.fromDegrees(
      pivotEncoder.getAbsolutePosition() * 360 
    ).plus(Rotation2d.fromDegrees(IntakeConstants.INTAKE_PIVOT_ENCODER_OFFSET_DEGREES));//-50
  }

  @Override
  public double getMeasurement() {
    return getAngleRelativeToGround().getDegrees();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    pivotMotor.set(MathUtil.clamp( output, -5, .5));
    SmartDashboard.putNumber("Intake Pivout out", output);
    SmartDashboard.putNumber("SETPOINT", setpoint.position);
    SmartDashboard.putNumber("CUR", getAngleRelativeToGround().getDegrees());
  }

  public boolean hasNote() {
    SmartDashboard.putBoolean("id 7 hit", intakeLimitSwitches[0].get());
    SmartDashboard.putBoolean("id 8 hit", intakeLimitSwitches[1].get());
    SmartDashboard.putBoolean("id 9 hit", intakeLimitSwitches[2].get());
    // for(DigitalInput intakeSwitch: intakeLimitSwitches){
    //   if (!intakeSwitch.get()){
    //     return true;
    //   }
    // }
    // return false;
    return !intakeLimitSwitches[1].get();
  }

  public boolean atGoal(){
    return this.getController().atGoal();
  }

  public Command startIntake() {
    return this.runOnce(()->{
      enable();
      setGoal(IntakeConstants.INTAKE_PIVOT_OUT_ANGLE);
      rollerMotor.set(IntakeConstants.INTAKE_ROLLER_INTAKE_SPEED);
    });
  }

  public Command retract() {
    return this.runOnce(()->{
      enable();
      setGoal(IntakeConstants.INTAKE_PIVOT_IN_ANGLE);
      rollerMotor.stopMotor();
    });
  }

  public Command fixNote() {
    return this.run(()->{
      rollerMotor.set(IntakeConstants.INTAKE_ROLLER_INTAKE_SPEED/2);
    }).onlyIf(()->!this.hasNote()).withTimeout(IntakeConstants.ROLLER_NOTEFIX_TIMEOUT).andThen(this.stopRoller());
  }

  public Command feed() { 
    return this.run(()->{
      System.out.println("FEED RUNNING" + System.currentTimeMillis());
      rollerMotor.set(IntakeConstants.INTAKE_ROLLER_FEED_SPEED);
    });
  }
  
  public Command ampMechFeed() { 
    return this.run(()->{
      System.out.println("FEED RUNNING" + System.currentTimeMillis());
      rollerMotor.set(IntakeConstants.INTAKE_ROLLER_AMP_MECH_FEED_SPEED);
    });
  }

  public Command stopRoller() {
    return this.runOnce(()->{
      rollerMotor.stopMotor();
    });
  }
}
