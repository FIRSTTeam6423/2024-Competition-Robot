package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IronUtil {

  public static class IronController extends CommandXboxController {

    private final double joystickDeadband, axisDeadband;

    /**
     * Creates a new Iron Controller
     *
     * @param port port controller is connected too
     * @param joystickDeadband Joystick deadzone
     * @param triggerDeadband Trigger deadzone
     */
    public IronController(int port, double joystickDeadband, double triggerDeadband) {
      super(port);

      this.joystickDeadband = joystickDeadband;
      this.axisDeadband = triggerDeadband;
    }

    /**
     * Joystick deadband
     *
     * @param rawAxis 0 is left Y, 1 is left X, 3 is Right Y, 4 is Right X //! might be wrong idk
     * @return double
     */
    public double joystickDeadbandOutput(int rawAxis) {
      return MathUtil.applyDeadband(
          Math.abs(Math.pow(super.getRawAxis(rawAxis), 2)) * Math.signum(super.getRawAxis(rawAxis)),
          0.025 // !!!!!!!!!!!!!!!!!!!!!!
          );
    }

    /**
     * Returns the circular angle of the joystick
     *
     * @param yAxis
     * @return Rotation2d
     */
    public Double flickStickOutput(int yAxis, int xAxis) {
      double y = joystickDeadbandOutput(yAxis);
      double x = joystickDeadbandOutput(xAxis);
      return (y == 0 && x == 0) ? 4242564 : Rotation2d.fromRadians(Math.atan2(y, x)).getRadians();
    }

    /**
     * Rumble Controller
     *
     * @param rmb
     * @param n
     * @return
     */
    public Command rumbleController(GenericHID.RumbleType rmb, double n) {
      return new InstantCommand(() -> super.getHID().setRumble(rmb, n));
    }
  }
}
