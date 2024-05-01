package frc.robot.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import static edu.wpi.first.units.MutableMeasure.mutable;

public class ShooterSim extends Shooter {
 
  // * Creates new shooter motor sims 
  // ! flywheel
  private final DCMotorSim leftMotor = new DCMotorSim(
    DCMotor.getNEO(ShooterConstants.LEFT_MOTOR), 
    1, // ! Gearing
    1.0 // Moment of inertia, should calculate that later
  );

  private final DCMotorSim rightMotor = new DCMotorSim(
    DCMotor.getNEO(ShooterConstants.LEFT_MOTOR), 
    1, // ! Gearing
    1.0 // Moment of inertia, should calculate that later
  );
  
  private final SimpleMotorFeedforward feedForward = 
    new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kV);

  private boolean enabled = false;

  private final MutableMeasure<Voltage> m_appliedVoltage = 
    mutable(Volts.of(0));

  private final MutableMeasure<Angle> m_angle = 
    mutable(Rotations.of(0));

  private final MutableMeasure<Velocity<Angle>> m_velocity = 
    mutable(RotationsPerSecond.of(0));

  private double goal = 0;

  

  protected ShooterSim() {}

  @Override
  public double getMeasurementLeft() {
    return leftMotor.getAngularVelocityRPM();
  }

  @Override
  public double getMeasurementRight() {
    return rightMotor.getAngularVelocityRPM();
  }

  @Override
  public void useOutputLeft(double output) {
    leftMotor.setInputVoltage(output);
  }

  @Override
  public void useOutputRight(double output) {
    rightMotor.setInputVoltage(output);
  }

  @Override
  public boolean atRPM() {
    // ! sheeeeesh
    if (
        (
          rightMotor.getAngularVelocityRPM() > ShooterConstants.SHOOT_RPM - 400 
          && 
          rightMotor.getAngularVelocityRPM() < goal + 400
        ) 
          && 
        (
          leftMotor.getAngularVelocityRPM() > ShooterConstants.SHOOT_RPM - 400 
          && 
          leftMotor.getAngularVelocityRPM() < goal + 400)
        ) 
    {
       return true; 
    }
    return false;
  }

  @Override
  public Command spinup() {
    return this.run( () -> {
      setGoal(ShooterConstants.SHOOT_RPM);
      this.enabled = true;
    });
  }

  @Override
  public void setGoal(double newGoal) {
    goal = newGoal;
  } 

  @Override
  public Command startSpinup() {
    return this.runOnce( () -> {
      this.enabled = true;
      goal = ShooterConstants.SHOOT_RPM;
    });
  }

  @Override
  public Command stopRollers() {
    return this.runOnce( () -> {
      this.enabled = false;
      leftMotor.setInputVoltage(0);
      rightMotor.setInputVoltage(0);
    });
  }

  @Override
  public Boolean rightTriggerPressed() {
    return RobotContainer.operator.getRightBumper();
  }

  @Override
  public Command feed() {
    return this.run( () -> {
      setGoal(ShooterConstants.AMP_MECH_FEED_SPEED);
      this.enabled = true;
    });
  }

  @Override
  public Command suckIn() {
    return this.run( () -> {
      setGoal(ShooterConstants.AMP_MECH_SUCK_IN_SPEED);
      this.enabled = true;
    });
  }

  @Override
  public Command suckBack() {
    return run( () -> {
      setGoal(ShooterConstants.AMP_MECH_SUCK_BACK_SPEED);
      this.enabled = true;
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getMeasurementLeft());

    if (enabled) {
      useOutputLeft(feedForward.calculate(goal/60));
      useOutputRight(feedForward.calculate(goal/60));
    }

    SmartDashboard.putBoolean("Trigger triggered", rightTriggerPressed());
  }

}
