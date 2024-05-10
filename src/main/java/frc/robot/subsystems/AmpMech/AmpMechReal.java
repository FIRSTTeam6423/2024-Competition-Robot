package frc.robot.subsystems.AmpMech;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpMechReal extends AmpMech {

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(AmpMechConstants.AMP_MECH_PIVOT_ENCODER);

  private CANSparkMax pivotMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_PIVOT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rollerMotor = new CANSparkMax(AmpMechConstants.AMP_MECH_ROLLER_MOTOR, MotorType.kBrushless);
  
  private DigitalInput beamBreak = new DigitalInput(AmpMechConstants.BEAM_BREAK);

  private boolean testAmpMechCode = false;
  
  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
    AmpMechConstants.AMP_MECH_PIVOT_kS, 
    AmpMechConstants.AMP_MECH_PIVOT_kG,
    AmpMechConstants.AMP_MECH_PIVOT_kV,
    AmpMechConstants.AMP_MECH_PIVOT_kA
  );
  

  protected AmpMechReal() { this.getController().setTolerance(15); }

  @Override
  public boolean atGoal() {
    return this.getController().atGoal();
  }

  @Override
  public boolean beamBreakHit() {
    return !beamBreak.get();
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
      double feedForward = pivotFeedForwardController.calculate(setpoint.position, setpoint.velocity);
      pivotMotor.set(feedForward + output);
      SmartDashboard.putBoolean("Beambreak", beamBreak.get());
      SmartDashboard.putBoolean("test code", testAmpMechCode);
  }

  public double getMeasurement() {
    return Rotation2d.fromDegrees(
      pivotEncoder.getAbsolutePosition() * 360
    ).times(-1).plus(
      Rotation2d.fromDegrees(AmpMechConstants.AMP_MECH_PIVOT_ENCODER_OFFSET_DEGREES)
    ).getDegrees();
  }

  @Override
  public Command prepareGrab() {
    return this.runOnce( () -> {
      enable();
      setGoal(AmpMechConstants.AMP_MECH_IN_ANGLE);
    });
  }

  @Override
  public Command suckNote() {
    return this.run( () -> {
      rollerMotor.set(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED);
    });
  }

  @Override
  public Command extend() {
    return this.run( () -> {
      enable();
      if (!testAmpMechCode) setGoal(AmpMechConstants.AMP_MECH_OUT_ANGLE);
      else setGoal(AmpMechConstants.AMP_MECH_OUT_ANGLE_TEST);
    });
  }

  @Override
  public Command deposit() {
    return this.run( () -> {
      rollerMotor.set(AmpMechConstants.AMP_MECH_DEPOSIT_SPEED);
    });
  }

  @Override
  public Command stopRollers() {
    return this.runOnce( () -> {
      rollerMotor.stopMotor();
    });
  }

  @Override
  public Command stow() {
    return this.runOnce( () -> {
      enable();
      setGoal(AmpMechConstants.AMP_MECH_STOW_ANGLE);
    });
  }

  @Override
  public Command suckIn() {
    return this.runOnce( () -> {
      rollerMotor.set(AmpMechConstants.SUCK_IN_SPEED);
    });
  }

  @Override
  public Command suckBack() {
    return this.run( () -> {
      rollerMotor.set(AmpMechConstants.SUCK_BACK_SPEED);
    });
  }

  @Override
  public Command switchCode() {
    return this.runOnce( () -> {
      testAmpMechCode = !testAmpMechCode;
    });
  }

}
