package frc.robot.subsystems.Drive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  static class GyroInputs {}

  void updateInputs(final GyroInputs inputs);

  /**
   * Gets gyro heading
   *
   * @return Rotation2d
   */
  Rotation2d getGyroHeading();

  /** Resets gyro */
  void resetGyro();
}
