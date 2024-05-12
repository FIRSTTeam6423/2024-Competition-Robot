package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Robot;
import java.util.function.Supplier;

public abstract class Intake extends ProfiledPIDSubsystem {

  private static final Intake instance;

  static {
    if (Robot.isReal()) {
      instance = new IntakeReal();
    } else {
      instance = new IntakeSim();
    }
  }

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
  }

  public static Intake getInstance() {
    return instance;
  }

  public abstract double getPivotVolts();

  public abstract Rotation2d getAngleRelativeToGround();

  public abstract double getMeasurement();

  public abstract void useOutput(double output, TrapezoidProfile.State setpoint);

  public abstract boolean hasNote();

  public abstract boolean triggerPressed();

  public abstract boolean fullyHasNote();

  public abstract boolean atGoal();

  public abstract Command setPivotVolts(Supplier<Double> volts);

  public abstract Command setVoltsRamp(double volts);

  public abstract Command startIntake();

  public abstract Command retract();

  public abstract Command fixNote();

  public abstract Command shooterFeed();

  public abstract Command ampMechFeed();

  public abstract Command stopRoller();

  public abstract Command startOutake();

  public abstract Command outakeRolling();

  public abstract Command unload();

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Intake angle", getAngleRelativeToGround().getDegrees());
  }
}
