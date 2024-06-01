package frc.robot.subsystems.Drive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

  private final Rotation2d rotation = new Rotation2d();

  @Override
  public void updateInputs(GyroInputs inputs) {}

  @Override
  public Rotation2d getGyroHeading() {
    return rotation;
  }

  @Override
  public void resetGyro() {}
}
