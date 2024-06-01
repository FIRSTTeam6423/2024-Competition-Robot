package frc.robot.subsystems.Climb;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Climb extends SubsystemBase {

  private final ClimbIO io;

  /*** initalizes Climb subsystem * @param io Hardware IO to use */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  /*** Checks if the climbers are at current limit * @return boolean */
  public boolean atCurrentLimit() {
    return io.getCurrent() > MAX_CURRENT_AMPS;
  }

  // * public

  /**
   * Runs climbers
   *
   * @param leftSupplier left Input supplier
   * @param rightSupplier right Input supplier
   * @return Command construct
   */
  public Command runClimb(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    return run(() -> {
          double leftInput, rightInput, leftOutput, rightOutput = 0;

          leftInput = leftSupplier.get();
          rightInput = rightSupplier.get();

          leftOutput = leftInput > 0 ? MAX_EXTEND_VOLTAGE : MAX_RETRACT_VOLTAGE;
          rightOutput = rightInput > 0 ? MAX_EXTEND_VOLTAGE : MAX_RETRACT_VOLTAGE;

          io.setVoltage(leftOutput, rightOutput);
        })
        .onlyIf(() -> !atCurrentLimit());
  }

  /**
   * Stop the climbers
   *
   * @return Command construct
   */
  public Command stopClimb() {
    return run(
        () -> {
          io.stopClimb();
        });
  }
}
