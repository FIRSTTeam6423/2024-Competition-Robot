package frc.robot.subsystems.AmpMech;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class AmpMech extends ProfiledPIDSubsystem {

  private static final AmpMech instance;

  static {
    if (Robot.isReal()) {
      instance = new AmpMechReal();
    } else {
      instance = new AmpMechSim();
    }
  }

  public static AmpMech getInstance() {
    return instance;
  }

  public AmpMech() {
    super(
      new ProfiledPIDController(
        AmpMechConstants.AMP_MECH_PIVOT_P,
        AmpMechConstants.AMP_MECH_PIVOT_I,
        AmpMechConstants.AMP_MECH_PIVOT_D,
        new TrapezoidProfile.Constraints(
          AmpMechConstants.AMP_MECH_MAX_VELOCITY_DEG_PER_SEC,
          AmpMechConstants.AMP_MECH_MAX_ACCELERATION_DEG_PER_SEC_SQUARED
        )
      ),
      0
    );
  }

  public abstract boolean atGoal();

  public abstract boolean beamBreakHit();


  @Override
  public abstract double getMeasurement();

  @Override
  public abstract void useOutput(double output, TrapezoidProfile.State setpoint);

  public abstract Command prepareGrab();

  public abstract Command suckNote();

  public abstract Command extend();

  public abstract Command deposit();

  public abstract Command stopRollers();

  public abstract Command stow();

  public abstract Command suckIn();

  public abstract Command suckBack();

  public abstract Command switchCode();

}
