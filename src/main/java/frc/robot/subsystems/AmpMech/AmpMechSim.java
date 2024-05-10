package frc.robot.subsystems.AmpMech;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpMechSim extends AmpMech {
 
  private final DCMotorSim pivotMotor = new DCMotorSim(
    DCMotor.getNEO(AmpMechConstants.AMP_MECH_PIVOT_MOTOR),
    1,
    1.0
  );
  private final DCMotorSim rollerMotor = new DCMotorSim(
    DCMotor.getNEO(AmpMechConstants.AMP_MECH_ROLLER_MOTOR),
    1,
    1.0
  );

  private DigitalInput beamBreak = new DigitalInput(AmpMechConstants.BEAM_BREAK);

  private boolean testAmpMechCode = false;

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
    AmpMechConstants.AMP_MECH_PIVOT_kS, 
    AmpMechConstants.AMP_MECH_PIVOT_kG,
    AmpMechConstants.AMP_MECH_PIVOT_kV,
    AmpMechConstants.AMP_MECH_PIVOT_kA
  );

  protected AmpMechSim() { this.getController().setTolerance(15); }

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
    pivotMotor.setInputVoltage(feedForward + output);
    SmartDashboard.putBoolean("Beambreak", beamBreak.get());
    SmartDashboard.putBoolean("Amp test mode?", testAmpMechCode);
  }

  public double getMeasurement() {
    return Rotation2d.fromDegrees(
      (pivotMotor.getAngularPositionRad() * 180/Math.PI) * 360
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
      rollerMotor.setInputVoltage(AmpMechConstants.AMP_MECH_ROLLER_SUCK_SPEED);
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
      rollerMotor.setInputVoltage(AmpMechConstants.AMP_MECH_DEPOSIT_SPEED);
    });
  }

  @Override 
  public Command stopRollers() {
    return this.runOnce( () -> { 
      rollerMotor.setInputVoltage(0); 
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
      rollerMotor.setInputVoltage(AmpMechConstants.SUCK_IN_SPEED);
    });
  }

  @Override
  public Command suckBack() {
    return this.run( () -> {
      rollerMotor.setInputVoltage(AmpMechConstants.SUCK_BACK_SPEED);
    });
  }

  @Override
  public Command switchCode() {
    return this.runOnce( () -> {
      testAmpMechCode = !testAmpMechCode;
    });
  }

}
