package frc.robot.commons;

public final class IronUtil {
    public static double deadzone(double input, double width){
		if(Math.abs(input) >= width){
			return input;
		} else {
			return 0;
		}
	}

	public static double squareInputKeepSign(double input) {
		double sign = Math.signum(input);
		return sign * Math.pow(input, 2);
	}
}