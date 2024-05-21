package frc.robot.subsystems.AmpMech;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.AmpMechConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpMechIOReal implements AmpMechIO {

  private final CANSparkMax rollerMotor, pivotMotor;
  private final DutyCycleEncoder pivotEncoder;

  private final DigitalOutput beamBreak = new DigitalOutput(BEAM_BREAK);

  public AmpMechIOReal() {
    rollerMotor = new CANSparkMax(AMP_MECH_ROLLER_MOTOR, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(AMP_MECH_PIVOT_MOTOR, MotorType.kBrushless);

    pivotEncoder = new DutyCycleEncoder(AMP_MECH_PIVOT_ENCODER);
  }

  @Override
  public void updateInputs(AmpMechInputs inputs) {
    inputs.ampMechAppliedVoltage = pivotMotor.getBusVoltage();
    inputs.ampMechCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.ampMechPosition =
        getAmpMechAngleRelativeToGround().getDegrees() * -1 + AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES;
  }

  @Override
  public Rotation2d getAmpMechAngleRelativeToGround() {
    return new Rotation2d();
  }

  @Override
  public Command setRollerSpeed(double speed) {
    return run(
        () -> {
          rollerMotor.set(speed);
        });
  }

  @Override
  public Command stopRoller() {
    return run(
        () -> {
          rollerMotor.stopMotor();
        });
  }

  @Override
  public Command setPivotSpeed(double speed) {
    return run(
        () -> {
          pivotMotor.set(speed);
        });
  }

  @Override
  public Rotation2d getPivotPosition() {
    return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition());
  }

  @Override
  public boolean getBeambreakStatus() {
    return beamBreak.get();
  }
}
