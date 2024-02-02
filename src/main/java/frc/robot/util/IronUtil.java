package frc.robot.util;

public final class IronUtil {
    public static double deadzone(double input, double zone) {
        if(Math.abs(input) >= zone){
			return input;
		} else {
			return 0;
		}
    }
}
