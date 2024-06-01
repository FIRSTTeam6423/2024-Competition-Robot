package frc.robot.subsystems.Drive.Module;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public abstract class ModuleIO {

  private final PIDController drivePID, pivotPID;
  private final SimpleMotorFeedforward driveFeedforward;
  private SwerveModuleState setpoint;

  private final String moduleID;

  public ModuleIO(String id) {
    this.moduleID = id;

    drivePID = new PIDController(MODULEDRIVE_P, MODULEDRIVE_I, MODULEDRIVE_D);

    driveFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    pivotPID = new PIDController(MODULEPIVOT_P, MODULEPIVOT_I, MODULEPIVOT_D);

    pivotPID.enableContinuousInput(-90, 90);
  }

  @AutoLog
  public static class ModuleInputs {
    public Rotation2d pivotRotation = new Rotation2d();
    public Rotation2d pivotPosition = new Rotation2d();
    public double pivotVelocity = 0.0;
    public double pivotAppliedVoltage = 0.0;

    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVoltage = 0.0;
  }

  public abstract void updateInputs(final ModuleInputs inputs);

  public final String getModuleID() {
    return this.moduleID;
  }
  ;

  /**
   * Gets Module state
   *
   * @return {@link SwerveModuleState}
   */
  public abstract SwerveModuleState getModuleState();

  /**
   * Gets Module position
   *
   * @return {@link SwerveModulePosition}
   */
  public abstract SwerveModulePosition getModulePosition();

  /**
   * Gets pivot motor's position
   *
   * @return {@link Rotation2d}
   */
  public abstract Rotation2d getPivotPose();

  /**
   * Gets drive motor's position
   *
   * @return {@link Rotation2d}
   */
  public abstract double getDrivePose();

  /**
   * Gets drive motor's velocity
   *
   * @return double
   */
  public abstract double getDriveVelocity();

  /**
   * Calculates pivot and drive motor outputs based on desired state
   *
   * @param state desired state
   */
  public void updateDesiredState(SwerveModuleState state) {
    setpoint = SwerveModuleState.optimize(state, getPivotPose());
    calculateDriveSpeed(setpoint.speedMetersPerSecond);
    calculatePivotAngle(setpoint.angle);
  }

  /**
   * Runs setpoint through Drive's PID controller
   *
   * @param speed
   */
  public void calculateDriveSpeed(double speed) {
    double volts =
        driveFeedforward.calculate(speed) + drivePID.calculate(getDriveVelocity(), speed);
    setDriveVoltage(volts);
  }

  /**
   * Runs setpoint through pivot's PID controller
   *
   * @param speed
   */
  public void calculatePivotAngle(Rotation2d rotation) {
    double volts = pivotPID.calculate(getPivotPose().getDegrees(), rotation.getDegrees());
    setPivotVoltage(volts);
  }

  /**
   * Runs voltage through the pivot motor
   *
   * @param voltage voltage
   */
  public abstract void setPivotVoltage(double voltage);

  /**
   * Runs voltage through the drive motor
   *
   * @param voltage voltage
   */
  public abstract void setDriveVoltage(double voltage);

  /** Resets the encoders */
  public abstract void resetEncoders();
}
