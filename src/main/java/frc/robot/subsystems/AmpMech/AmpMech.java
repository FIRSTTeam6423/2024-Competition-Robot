package frc.robot.subsystems.AmpMech;

import static frc.robot.Constants.AmpMechConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class AmpMech extends ProfiledPIDSubsystem {

  private final AmpMechIO io;

  private final ArmFeedforward pivotFeedforward;

  /**
   * initalizes AmpMech subsystem
   * 
   * @param io Hardware IO to use 
   */
  public AmpMech(AmpMechIO io) {
    super(
        new ProfiledPIDController(
            AMP_MECH_PIVOT_P,
            AMP_MECH_PIVOT_I,
            AMP_MECH_PIVOT_D,
            new TrapezoidProfile.Constraints(
                AMP_MECH_MAX_VELOCITY_DEG_PER_SEC, AMP_MECH_MAX_ACCELERATION_DEG_PER_SEC_SQUARED)));
    this.io = io;

    pivotFeedforward =
        new ArmFeedforward(
            AMP_MECH_PIVOT_kS, AMP_MECH_PIVOT_kG, AMP_MECH_PIVOT_kV, AMP_MECH_PIVOT_kA);

    this.getController().setTolerance(15);
  }

  /**
   * Gets motor position in degrees 
   * 
   * @return double 
   */
  @Override
  public double getMeasurement() {
    return io.getAmpMechAngleRelativeToGround().getDegrees() * -1
        + AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES;
  }

  /***
   * Uses PID controller output
   * 
   * @param output
   * @param setpoint
   */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);
    io.setPivotSpeed(feedforward + output);
    SmartDashboard.putBoolean("Beambreak", io.getBeambreakStatus());
  }

  // * public

  /**
   * Is the PID controller at goal
   *
   * @return boolean
   */
  public boolean atGoal() {
    return getController().atGoal();
  }

  /**
   * Is the beam break triggered?
   *
   * @return boolean
   */
  public boolean beamBreakTriggered() {
    return io.getBeambreakStatus();
  }

  /**
   * Sets Trapezoid Profile goal
   *
   * @param goal goal state
   * @return {@link Command}
   */
  public Command setPivotGoal(double goal) {
    return runOnce(
        () -> {
          enable();
          setGoal(goal);
        });
  }

  /**
   * Runs rollers at a certain speed
   *
   * @param speed roller speed
   * @return {@link Command}
   */
  public Command runRoller(double speed) {
    return run(() -> io.setRollerSpeed(speed));
  }

  /**
   * Stops roller motor
   * 
   * @return {@link Command}
   */
  public Command stopRollers() {
    return runOnce(
        () -> {
          io.stopRoller();
        });
  }
}
