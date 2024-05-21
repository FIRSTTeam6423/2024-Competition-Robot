package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;
import java.util.function.Supplier;

public class ClimbCommands {

  private final Climb climb;

  public ClimbCommands(Climb climb) {
    this.climb = climb;
  }

  /**
   * Stop the climbers
   *
   * @return Command construct
   */
  public Command stopClimb() {
    return run(() -> climb.setClimbVoltage(0, 0));
  }

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

          climb.setClimbVoltage(leftOutput, rightOutput);
        })
        .onlyIf(() -> !climb.atCurrentLimit());
  }
}
