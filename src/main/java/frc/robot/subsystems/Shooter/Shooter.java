package frc.robot.subsystems.Shooter;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  private final SimpleMotorFeedforward feedforward;
  private double goal;
  private boolean enabled;

  public Shooter(ShooterIO io) {

    this.io = io;

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    goal = 0.0;
    enabled = false;
  }

  public boolean atRPM() {
    double velocity = io.getVelocityLeft();
    return velocity > SHOOT_RPM - 400 && velocity < SHOOT_RPM + 400;
  }

  public Command setGoal(double goal) {
    return run(
        () -> {
          enabled = true;
          this.goal = goal;
        });
  }

  public Command stopShooter() {
    return runOnce(
        () -> {
          enabled = false;
          io.stopMotors();
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shot RPM", io.getVelocityLeft());

    if (enabled) {
      double output = feedforward.calculate(goal / 60);
      io.setMotorVoltage(output, output);
    }
  }
}
