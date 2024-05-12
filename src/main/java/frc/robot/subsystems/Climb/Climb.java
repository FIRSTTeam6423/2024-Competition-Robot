package frc.robot.subsystems.Climb;

import static frc.robot.subsystems.Climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private final ClimbIO io;

  /*** initalizes Climb subsystem * @param io Hardware IO to use */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  /***
   * Sets voltage of climbers
   *
   * @param leftVoltage
   *            voltage to run through left motor
   * @param rightVoltage
   *            voltage to run through right motor
   * @return Command construct
   */
  public Command setClimbVoltage(double leftVoltage, double rightVoltage) {
    return run(
        () -> {
          io.setVoltage(leftVoltage, rightVoltage);
          if (leftVoltage == 0 && rightVoltage == 0) io.stopClimb();
        });
  }

  /*** Checks if the climbers are at current limit * @return boolean */
  public boolean atCurrentLimit() {
    return io.getCurrent() > MAX_CURRENT_AMPS;
  }
}
