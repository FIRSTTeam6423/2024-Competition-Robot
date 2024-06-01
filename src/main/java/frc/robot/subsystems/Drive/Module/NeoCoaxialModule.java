package frc.robot.subsystems.Drive.Module;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NeoCoaxialModule extends ModuleIO {

  private final CANSparkMax driveMotor, pivotMotor;

  private final RelativeEncoder driveEncoder;
  private final DutyCycleEncoder pivotEncoder;
  private final Rotation2d pivotOffset;

  public NeoCoaxialModule(
      String moduleID,
      Rotation2d pivotOffset,
      int pivotMotorID,
      int driveMotorID,
      int pivotEncoderID,
      boolean driveInverted,
      boolean pivotInverted) {

    super(moduleID);

    // * Drive config
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    driveMotor.setInverted(driveInverted);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    driveMotor.burnFlash();

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0);
    driveEncoder.setPositionConversionFactor(DRIVE_ROTATIONS_TO_METERS);
    driveEncoder.setVelocityConversionFactor(RPM_TO_METERS_PER_SEC);
    driveEncoder.setMeasurementPeriod(1); // 32

    // * Pivot config

    pivotMotor = new CANSparkMax(pivotEncoderID, MotorType.kBrushless);
    pivotMotor.setInverted(pivotInverted);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setSmartCurrentLimit(PIVOT_MOTOR_CURRENT_LIMIT); // !

    pivotEncoder = new DutyCycleEncoder(pivotEncoderID);
    pivotEncoder.reset();
    this.pivotOffset = pivotOffset;
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.pivotRotation = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition());
    inputs.pivotPosition = getPivotPose();
    inputs.pivotVelocity = pivotEncoder.getDistancePerRotation();
    inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput();

    inputs.drivePosition = getDrivePose();
    inputs.driveVelocity = getDriveVelocity();
    inputs.driveAppliedVoltage = driveMotor.getAppliedOutput();
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getDriveVelocity(), getPivotPose());
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDrivePose(), getPivotPose());
  }

  @Override
  public Rotation2d getPivotPose() {
    return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition()).minus(pivotOffset);
  }

  @Override
  public double getDrivePose() {
    return driveEncoder.getPosition();
  }

  @Override
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    pivotEncoder.reset();
  }
}
