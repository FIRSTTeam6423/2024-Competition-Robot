package frc.robot.subsystems.Drive.Gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {

  private final AHRS gyro;

  public GyroIONavX() {
    gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {}

  @Override
  public Rotation2d getGyroHeading() {
    return gyro.getRotation2d();
  }

  @Override
  public void resetGyro() {
    gyro.reset();
  }
}