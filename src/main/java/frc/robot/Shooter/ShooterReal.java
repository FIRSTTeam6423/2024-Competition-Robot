package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.commons.IronUtil;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;


public class ShooterReal extends Shooter {
  /** Creates a new Shooter. */
  private CANSparkMax leftMotor = new CANSparkMax(ShooterConstants.LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(ShooterConstants.RIGHT_MOTOR, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV,
      ShooterConstants.kA); //tuned for rotations / second

  private boolean enabled = false;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private double goal = 0;

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          (Measure<Voltage> volts) -> {
            leftMotor.setVoltage(volts.in(Volts));
            rightMotor.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("shooter-wheel")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(leftEncoder.getPosition(), Rotations))
                .angularVelocity(
                    m_velocity.mut_replace(leftEncoder.getVelocity() * 60, RotationsPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("shooter")
          this));

  protected ShooterReal() {
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
  }

  public Command runQuasistatic(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command runDynamic(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

  @Override
  public double getMeasurementLeft() {
    return leftEncoder.getVelocity(); // rpm
  }

  @Override
  public double getMeasurementRight() {
    return rightEncoder.getVelocity();
  }

  @Override
  public void useOutputLeft(double output) {
    leftMotor.setVoltage(output);
  }

  @Override
  public void useOutputRight(double output) {
    rightMotor.setVoltage(output);
  }

  @Override
  public boolean atRPM() {
    return IronUtil.inRange(getMeasurementLeft(), ShooterConstants.SHOOT_RPM, 400);
  }

  @Override
  public Command spinup() {
    return this.run(() -> {
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
    return this.runOnce(() -> {
      this.enabled = true;
      goal = ShooterConstants.SHOOT_RPM;
    });
  }

  @Override
  public Command stopRollers() {
    return this.runOnce(() -> {
      this.enabled = false;
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  @Override
  public Boolean rightTriggerPressed(){
    return RobotContainer.operator.getRightBumper();
  }

  @Override
  public Command feed() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_FEED_SPEED);

    });
  }

  @Override
  public Command feedSlow() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_FEED_SPEED);
      this.enabled = true;
    });

  }

  @Override
  public Command suckIn() {
    return this.run(() -> {
      setGoal(ShooterConstants.AMP_MECH_SUCK_IN_SPEED);
      this.enabled = true;
    });
  }

  @Override
  public Command suckBack(){
    return run(()->{
      setGoal(ShooterConstants.AMP_MECH_SUCK_BACK_SPEED);
      this.enabled = true;
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shot RPM", getMeasurementLeft());

    if (enabled) {
      useOutputLeft(
        //leftController.calculate(getMeasurementLeft()/60, goal/60) + 
          feedForward.calculate(goal/60));
      useOutputRight(
          feedForward.calculate(goal/60));
    }

    SmartDashboard.putBoolean("Trigger triggered", rightTriggerPressed());
  }

}
