package frc.robot.commons;

public final class IronUtil {
  public static double deadzone(double input, double zone) {
    if (Math.abs(input) >= zone) {
      return input;
    } else {
      return 0;
    }
  }

  public static double powKeepSign(double input, double pow) {
    int sign = (int) Math.signum(input);
    return sign * Math.pow(input, pow);
  }
}
