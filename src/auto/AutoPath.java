package auto;

import math.Point;
import trajectory.RobotConstraints;
import trajectory.TrajectoryGenerator;
import trajectory.TrajectoryPoint;

public class AutoPath {
	
	private final TrajectoryPoint[][] traj;
	private final TimeRangedCommand[] commands;

	public AutoPath(RobotConstraints rc, boolean isReversed, TimeRangedCommand[] commands, Point... path) {
		traj = TrajectoryGenerator.generate(rc, isReversed, "", path);
		if(commands == null) {
			this.commands =  new TimeRangedCommand[0];
		} else {
			this.commands = commands;
		}
	}
	
	public final TrajectoryPoint[] getLeftTrajectory() {
		return traj[0];
	}
	
	public final TrajectoryPoint[] getRightTrajectory() {
		return traj[1];
	}
	
	public final TimeRangedCommand[] getCommands() {
		return commands;
	}
}
