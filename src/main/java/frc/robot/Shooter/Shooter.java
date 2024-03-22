// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LEDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Intake.Intake;
import frc.robot.commons.IronUtil;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();




  private PIDController leftController = new PIDController(ShooterConstants.LEFT_ROLLER_P,
      ShooterConstants.LEFT_ROLLER_I, ShooterConstants.LEFT_ROLLER_D);
  private PIDController rightController = new PIDController(ShooterConstants.RIGHT_ROLLER_P,
      ShooterConstants.RIGHT_ROLLER_I, ShooterConstants.RIGHT_ROLLER_D);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV,
      ShooterConstants.kA); //tuned for rotations / second

  private boolean enabled = false;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private double goal = 0;

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          (Measure<Voltage> volts) -> {
            leftMotor.setVoltage(volts.in(Volts));
            rightMotor.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("shooter-wheel")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(leftEncoder.getPosition(), Rotations))
                .angularVelocity(
                    m_velocity.mut_replace(leftEncoder.getVelocity() * 60, RotationsPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("shooter")
          this));

  public Shooter() {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
  }

  public Command runQuasistatic(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command runDynamic(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

  public double getMeasurementLeft() {
    return leftEncoder.getVelocity(); // rpm
  }

  public double getMeasurementRight() {
    return rightEncoder.getVelocity();
  }

  public void useOutputLeft(double output) {
    leftMotor.setVoltage(output);
  }

  public void useOutputRight(double output) {
    rightMotor.setVoltage(output);
  }

  public boolean atRPM() {
    return IronUtil.inRange(getMeasurementLeft(), ShooterConstants.SHOOT_RPM, 400);
  }

  public Command spinup() {
    return this.run(() -> {
      setGoal(ShooterConstants.SHOOT_RPM);
      enable();
    });
  }

  private void enable() {
    this.enabled = true;
  }

  private void disable() {
    this.enabled = false;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private void setGoal(double newGoal) {
    goal = newGoal;
  }

  public Command startSpinup() {
    return this.runOnce(() -> {
      enable();
      goal = ShooterConstants.SHOOT_RPM;
    });
  }

  public Command stopRollers() {
    return this.runOnce(() -> {
      disable();
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  public Boolean rightTriggerPressed(){
    return RobotContainer.getOperatorRightTrigger();
  }

  public Command feed() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_FEED_SPEED);
      enable();
    });
  }

  public Command feedSlow() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_FEED_SPEED);
      enable();
    });

  }

  public Command suckIn() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_SUCK_IN_SPEED);
      System.err.println("bruh moment");
      enable();
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shot RPM",getMeasurementLeft());

    if (enabled) {
      useOutputLeft(
        //leftController.calculate(getMeasurementLeft()/60, goal/60) + 
          feedForward.calculate(goal/60));
      useOutputRight(
          feedForward.calculate(goal/60));
    }

    SmartDashboard.putBoolean("Trigger triggered", rightTriggerPressed());
  }

  public Command suckBack(){
    return run(()->{
      setGoal(ShooterConstants.AMP_MECH_SUCK_BACK_SPEED);
      enable();
    });
  }

}
