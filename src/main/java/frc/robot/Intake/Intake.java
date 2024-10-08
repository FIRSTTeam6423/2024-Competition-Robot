// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends ProfiledPIDSubsystem {

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER);

  private CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR, MotorType.kBrushless);

  private RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
      IntakeConstants.PIVOT_kS,
      IntakeConstants.PIVOT_kG,
      IntakeConstants.PIVOT_kV,
      IntakeConstants.PIVOT_kA);

  private DigitalInput[] intakeLimitSwitches = {
      new DigitalInput(7), // TODO gotta fix outputs
      new DigitalInput(9),
      new DigitalInput(8),
  };

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
  private final MutableMeasure<Velocity<Angle>> m_angularVelocity = mutable(DegreesPerSecond.of(0));

  SysIdRoutine m_sysIdRoutine;

  private double voltRamp = 0;
  private double voltageAdjustment = 2;
  private double voltRampCounter = 0;
  private double voltRampCheckTicks = 6; 
  /** Creates a new Intake. */
  public Intake() {
    super(
        new ProfiledPIDController(
            IntakeConstants.PIVOT_P,
            IntakeConstants.PIVOT_I,
            IntakeConstants.PIVOT_D,
            new TrapezoidProfile.Constraints( 
                IntakeConstants.MAX_VELOCITY_DEG_PER_SEC,
                IntakeConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED)),
        0);

    pivotMotor.setInverted(true);
    getController().setTolerance(5);
  }

  private double getPivotVolts() {
    return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  private Rotation2d getAngleRelativeToGround() {
    return Rotation2d.fromDegrees(
        pivotEncoder.getAbsolutePosition() * 360)
        .plus(Rotation2d.fromDegrees(IntakeConstants.PIVOT_ENCODER_OFFSET_DEGREES));// -50
  }

  @Override
  public double getMeasurement() {
    return getAngleRelativeToGround().getDegrees();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double combinedOutput = output + pivotFeedForwardController.calculate(Units.degreesToRadians(setpoint.position),
        Units.degreesToRadians(setpoint.velocity));
    pivotMotor.set(MathUtil.clamp(combinedOutput, -1, 1));
  }

  public boolean hasNote() {
    SmartDashboard.putBoolean("Intake Lim 7", intakeLimitSwitches[0].get());
    SmartDashboard.putBoolean("Intake Lim 8", intakeLimitSwitches[1].get());
    SmartDashboard.putBoolean("Intake Lim 9", intakeLimitSwitches[2].get());
    for (DigitalInput intakeSwitch : intakeLimitSwitches) {
      if (!intakeSwitch.get()) {
        return true;
      }
    }
    return false;
    // return !intakeLimitSwitches[1].get();
  }

  public boolean triggerPressed(){
    return RobotContainer.getOperatorRightTrigger();
  }

  public boolean fullyHasNote() {
    return (!intakeLimitSwitches[1].get()) || (!intakeLimitSwitches[0].get() && !intakeLimitSwitches[2].get());
  }

  public boolean atGoal() {
    return this.getController().atGoal();
  }

  public Command setPivotVolts(Supplier<Double> volts) {
    return this.runOnce(() -> {
      pivotMotor.setVoltage(volts.get());
    });
  }

  public Command setVoltsRamp(double volts) {
    return runOnce(()->{
      voltRamp = volts;
    }).andThen(run(()->{
      if (voltRampCounter >= voltRampCheckTicks) {
        voltRampCounter = 0;
        if (rollerEncoder.getVelocity() <= 5) {
          if(volts>0){
            voltRamp += voltageAdjustment;
          }
          else{
            voltRamp -= voltageAdjustment;
          }
        } else {
          voltRamp = volts;
        }
      } else {
        voltRampCounter ++;
      }
      rollerMotor.setVoltage(voltRamp);
    }));
  }

  public Command startIntake() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_OUT_ANGLE);
      //rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
    }).andThen(setVoltsRamp(IntakeConstants.ROLLER_INTAKE_SPEED)).until(this::fullyHasNote).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  

  public Command retract() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_IN_ANGLE);
      rollerMotor.stopMotor();
    });
  }

  public Command fixNote() {
    return this.run(() -> {
      rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED / 2);
    }).onlyIf(() -> !this.hasNote()).withTimeout(IntakeConstants.ROLLER_NOTEFIX_TIMEOUT).andThen(this.stopRoller());
  }

  public Command shooterFeed() {
    return this.runOnce(() -> {
      //rollerMotor.set(IntakeConstants.ROLLER_FEED_SHOOTER_SPEED);
    }).andThen(setVoltsRamp(IntakeConstants.ROLLER_FEED_SHOOTER_SPEED));
  }

  public Command ampMechFeed() {
    return this.runOnce(() -> {
      //rollerMotor.set(IntakeConstants.ROLLER_AMP_MECH_FEED_SPEED);
    }).andThen(setVoltsRamp(IntakeConstants.ROLLER_AMP_MECH_FEED_SPEED));
  }

  public Command stopRoller() {
    return this.runOnce(() -> {
      rollerMotor.stopMotor();
    });
  }

  public Command startOutake() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_HORIZONTAL_ANGLE);
    });
  }

  public Command outakeRolling() {
    return this.run(() -> {
      rollerMotor.set(IntakeConstants.ROLLER_OUTAKE_SPEED);
    });
  }

  public Command suckBack(){
    return run(()->{
      rollerMotor.set(IntakeConstants.SUCK_BACK_SPEED);
    });
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putNumber("intake angle", getAngleRelativeToGround().getDegrees());
    SmartDashboard.putNumber("raw encoder value", pivotEncoder.getAbsolutePosition());
  }

  

}
