package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  static class ClimbInputs {
    public double averageCurrent = 0.0;
    public double leftRotations = 0.0;
    public double rightRotations = 0.0;
    public double appliedVoltageLeft = 0.0;
    public double appliedVoltageRight = 0.0;
  }

  void updateInputs(final ClimbInputs inputs);

  /**
   * Stops climb motors
   *
   * @return {@link Command}
   */
  Command stopClimb();

  /**
   * Sets voltage of climb motors
   *
   * @param leftVoltage
   * @param rightVoltage
   * @return {@link Command}
   */
  Command setVoltage(double leftVoltage, double rightVoltage);

  /*** Gets average current of climb motors
   *
   * @return double
   */
  double getCurrent();
}
