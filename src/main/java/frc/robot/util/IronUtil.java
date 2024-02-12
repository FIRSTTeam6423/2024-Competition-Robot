package frc.robot.util;

import frc.robot.Constants;

public final class IronUtil {
    public static double deadzone(double input, double zone) {
        if(Math.abs(input) >= zone){
			return input;
		} else {
			return 0;
		}
    }
	public static boolean inRange(double input, double goal, double range) {
		return input > goal - range && input < goal + range;
	} 
}
