package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  static class ShooterInputs {
    public double leftVelocity = 0.0;
    public double rightVelocity = 0.0;
    public double leftAppliedVoltage = 0.0;
    public double rightAppliedVoltage = 0.0;
    public double leftAppliedCurrent = 0.0;
    public double rightAppliedCurrent = 0.0;
  }

  void updateInputs(ShooterInputs inputs);

  public Command setMotorVoltage(double leftVoltage, double rightVoltage);

  public Command stopMotors();

  public double getVelocityLeft();

  public double getVelocityRight();
}
