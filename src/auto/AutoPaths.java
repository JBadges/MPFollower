package auto;

import org.usfirst.frc.team610.robot.Constants;

import math.Point;

public class AutoPaths {

	private static AutoPath centerStartLeftSwitch = new AutoPath(Constants.rc, false, null, new Point(0,0,0), new Point(3.3, -1.12912, 0));
	
	public static AutoPath getCenterStartLeftSwitch() {
		return centerStartLeftSwitch;
	}
	
	private static AutoPath centerStartRightSwitch = new AutoPath(Constants.rc, false, null, new Point(0,0,0), new Point(3.3, 1.12912, 0));
	
	public static AutoPath getCenterStartRightSwitch() {
		return centerStartRightSwitch;
	}
	
}
