package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

public class IntakeSim extends Intake {

  private DCMotorSim pivotMotor = new DCMotorSim(
    DCMotor.getNEO(IntakeConstants.PIVOT_MOTOR),
    1,
    1.0
  );
  private DCMotorSim rollerMotor = new DCMotorSim(
    DCMotor.getNEO(IntakeConstants.ROLLER_MOTOR),
    1,
    1.0
  );

  private ArmFeedforward pivotFeedForwardController = new ArmFeedforward(
    IntakeConstants.PIVOT_kS,
    IntakeConstants.PIVOT_kG,
    IntakeConstants.PIVOT_kV,
    IntakeConstants.PIVOT_kA
  );


  private DigitalInput[] intakeLimitSwitches = {
    new DigitalInput(7), // ! TODO argh
    new DigitalInput(9),
    new DigitalInput(8),
  };

  SysIdRoutine m_sysIdRoutine;

  private double voltRamp = 0;
  private double voltageAdjustment = 2;
  private double voltRampCounter = 0;
  private double voltRampCheckTicks = 6; 

  protected IntakeSim() {
    getController().setTolerance(5);
  }

  @Override
  public double getPivotVolts() {
    return pivotMotor.getOutput().mean();
  }

  @Override
  public Rotation2d getAngleRelativeToGround() {
    return Rotation2d.fromDegrees((pivotMotor.getAngularPositionRad() * 180/Math.PI) * 360)
      .plus(
        Rotation2d.fromDegrees(IntakeConstants.PIVOT_ENCODER_OFFSET_DEGREES)
      );
  }

  @Override
  public double getMeasurement() {
    return getAngleRelativeToGround().getDegrees();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double combinedOutput = output + pivotFeedForwardController.calculate(Units.degreesToRadians(setpoint.position),
          Units.degreesToRadians(setpoint.velocity));
    pivotMotor.setInputVoltage(MathUtil.clamp(combinedOutput, -1, 1));
  }

  @Override
  public boolean hasNote() {
    SmartDashboard.putBoolean("Intake Lim 7", intakeLimitSwitches[0].get());
    SmartDashboard.putBoolean("Intake Lim 8", intakeLimitSwitches[1].get());
    SmartDashboard.putBoolean("Intake Lim 9", intakeLimitSwitches[2].get());
    for (DigitalInput intakeSwitch : intakeLimitSwitches) {
      if (!intakeSwitch.get()) {
        return true;
      }
    }
    return false;
    // return !intakeLimitSwitches[1].get();
  }
  
  @Override
  public boolean triggerPressed(){
    return RobotContainer.operator.getRightBumper();
  }

  @Override
  public boolean fullyHasNote() {
    return true;
    //return (!intakeLimitSwitches[1].get()) || (!intakeLimitSwitches[0].get() && !intakeLimitSwitches[2].get());
  }

  @Override
  public boolean atGoal() {
    return true;
    //return this.getController().atGoal();
  }

  @Override
  public Command setPivotVolts(Supplier<Double> volts) {
    return this.runOnce(() -> {
      pivotMotor.setInputVoltage(volts.get());
    });
  }

  @Override
  public Command setVoltsRamp(double volts) {
    return runOnce(()->{
      voltRamp = volts;
    }).andThen(run(()->{
      if (voltRampCounter >= voltRampCheckTicks) {
        voltRampCounter = 0;
        if (rollerMotor.getAngularVelocityRPM() <= 5) { // ! I think this should be fine?
          if(volts>0){
            voltRamp += voltageAdjustment;
          }
          else{
            voltRamp -= voltageAdjustment;
          }
        } else {
          voltRamp = volts;
        }
      } else {
        voltRampCounter ++;
      }
      rollerMotor.setInputVoltage(voltRamp);
    }));
  }

  @Override
  public Command startIntake() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_OUT_ANGLE);
      //rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
    }).andThen(setVoltsRamp(IntakeConstants.ROLLER_INTAKE_SPEED)).until(this::fullyHasNote).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  } 

  @Override
  public Command retract() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_IN_ANGLE);
      rollerMotor.setInputVoltage(0);
    });
  }

  @Override
  public Command fixNote() {
    return this.run(() -> {
      rollerMotor.setInputVoltage(IntakeConstants.ROLLER_INTAKE_SPEED / 2);
    }).onlyIf(() -> !this.hasNote()).withTimeout(IntakeConstants.ROLLER_NOTEFIX_TIMEOUT).andThen(this.stopRoller());
  }

  @Override
  public Command shooterFeed() {
    return this.runOnce( () -> { setVoltsRamp(IntakeConstants.ROLLER_FEED_SHOOTER_SPEED); });
  }

  @Override
  public Command ampMechFeed() {
    return this.runOnce( () -> { setVoltsRamp(IntakeConstants.ROLLER_AMP_MECH_FEED_SPEED); });
  }

  @Override
  public Command stopRoller() {
    return this.runOnce(() -> {
      rollerMotor.setInputVoltage(0);
    });
  }

  @Override
  public Command startOutake() {
    return this.runOnce(() -> {
      enable();
      setGoal(IntakeConstants.PIVOT_HORIZONTAL_ANGLE);
    });
  }

  @Override
  public Command outakeRolling() {
    return this.run(() -> {
      rollerMotor.setInputVoltage(IntakeConstants.ROLLER_OUTAKE_SPEED);
    });
  }

  @Override
  public Command unload(){
    return run(()->{
      rollerMotor.setInputVoltage(IntakeConstants.SUCK_BACK_SPEED);
    });
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putNumber("intake angle", getAngleRelativeToGround().getDegrees());
  }

}
