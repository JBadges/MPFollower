package auto;

public class TimeRangedCommand {

	private final double startTime;
	private final double endTime;
	private final SingleCommand c;

	public TimeRangedCommand(double startTime, double endTime, SingleCommand c) {
		this.startTime = startTime;
		this.endTime = endTime;
		this.c = c;
	}
	
	//To be called periodically with the time since the start of the path in seconds
	public void tryAtTime(double time) {
		if(time >= startTime && time < endTime) {
			c.start();
		}
		if(time >= endTime) {
			c.end();
		}
	}
	
}

abstract class SingleCommand {
	private boolean hasRan = false;
	public final void start() {
		if(!hasRan) {
			run();
			hasRan = true;
		}
	}
	public abstract void run();
	private boolean hasEnded = false;
	public final void end() {
		if(!hasEnded) {
			onEnd();
			hasEnded = true;
		}
	}
	public abstract void onEnd();
}