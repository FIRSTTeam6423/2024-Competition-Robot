// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import java.util.function.Supplier;

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

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER);

  private CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR, MotorType.kBrushless);

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
      IntakeConstants.PIVOT_kS, 
      IntakeConstants.PIVOT_kG,
      IntakeConstants.PIVOT_kV,
      IntakeConstants.PIVOT_kA
    );

  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(7), // TODO gotta fix outputs
    new DigitalInput(9),
    new DigitalInput(8),
  };
  
  /** Creates a new Intake. */
  public Intake() {
    super(
      new ProfiledPIDController(
            IntakeConstants.PIVOT_P,
            IntakeConstants.PIVOT_I,
            IntakeConstants.PIVOT_D,
            new TrapezoidProfile.Constraints(
                IntakeConstants.MAX_VELOCITY_DEG_PER_SEC,
                IntakeConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED
            )
      ),
      0
    );
    pivotMotor.setInverted(true);
  }

  private Rotation2d getAngleRelativeToGround() {
     return Rotation2d.fromDegrees(
      pivotEncoder.getAbsolutePosition() * 360 
    ).plus(Rotation2d.fromDegrees(IntakeConstants.PIVOT_ENCODER_OFFSET_DEGREES));//-50
  }

  @Override
  public double getMeasurement() {
    return getAngleRelativeToGround().getDegrees();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double combinedOutput=output+pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
    pivotMotor.set(MathUtil.clamp( combinedOutput, -5, .5));
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

  public Command setPivotVolts(Supplier<Double> volts) {
    return this.runOnce(()->{
      pivotMotor.setVoltage(volts.get());
    });
  }

  public Command startIntake() {
    return this.runOnce(()->{
      enable();
      setGoal(IntakeConstants.PIVOT_OUT_ANGLE);
      rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
    });
  }

  public Command retract() {
    return this.runOnce(()->{
      enable();
      setGoal(IntakeConstants.PIVOT_IN_ANGLE);
      rollerMotor.stopMotor();
    });
  }

  public Command fixNote() {
    return this.run(()->{
      rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED/2);
    }).onlyIf(()->!this.hasNote()).withTimeout(IntakeConstants.ROLLER_NOTEFIX_TIMEOUT).andThen(this.stopRoller());
  }

  public Command shooterFeed() { 
    return this.run(()->{
      rollerMotor.set(IntakeConstants.ROLLER_FEED_SHOOTER_SPEED);
    });
  }
  
  public Command ampMechFeed() { 
    return this.run(()->{
      rollerMotor.set(IntakeConstants.ROLLER_AMP_MECH_FEED_SPEED);
    });
  }

  public Command stopRoller() {
    return this.runOnce(()->{
      rollerMotor.stopMotor();
    });
  }

public Command startOutake() {
    return this.runOnce(()->{
      enable();
      setGoal(IntakeConstants.PIVOT_HORIZONTAL_ANGLE);
    });
  }

public Command outakeRolling() {
    return this.run(()->{
      rollerMotor.set(IntakeConstants.ROLLER_OUTAKE_SPEED);
    });
  }
}
