package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Climb.Climb;


public class ClimbCommands {

  public static Command climbChain(XboxController controller, Climb climb) {
    return Commands.run( () -> {
      climb.setVoltage(controller::getRightY, controller::getLeftY);
    });
  }

  public static Command climbStop(Climb climb) {
    return Commands.runOnce( () -> {
      climb.StopClimb();
    });
  }
}
