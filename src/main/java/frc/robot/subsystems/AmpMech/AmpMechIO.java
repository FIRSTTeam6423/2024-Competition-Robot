package frc.robot.subsystems.AmpMech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface AmpMechIO {

  @AutoLog
  static class AmpMechInputs {
    public double ampMechAppliedVoltage = 0.0;
    public double ampMechCurrentAmps = 0.0;
    public double ampMechPosition = 0.0;
  }

  void updateInputs(final AmpMechInputs inputs);

  /**
   * Gets AmpMech angle 
   * 
   * @return {@link Rotation2d} 
   */
  Rotation2d getAmpMechAngleRelativeToGround();

  /**
   * Set roller motor speed
   *
   * @param speed speeeeeed
   * @return {@link Command}
   */
  Command setRollerSpeed(double speed);

  /**
   * Stops roller motors
   *  
   * @return {@link Command}
   */
  Command stopRoller();

  /**
   * Sets the pivot motor's speed
   *
   * @param speed speeeeeeeeeeeeeeeeeeeed
   * @return {@link Command}
   */
  Command setPivotSpeed(double speed);

  /**
   * Gets pivot motor's position 
   * 
   * @return {@link Rotation2d}
   */
  Rotation2d getPivotPosition();

  /**
   * Gets beambreak position (will always return true in sim) 
   * 
   * @return boolean
   */
  boolean getBeambreakStatus();
}
