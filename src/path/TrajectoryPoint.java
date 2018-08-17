package path;

public class TrajectoryPoint {
    private double time;
    private double pos;
    private double vel;
    private double acc;
    private double heading;
    private double timeStamp;

    public TrajectoryPoint(double time, double pos, double vel, double acc, double heading, double timeStamp) {
        this.time = time;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
        this.heading = heading;
        this.timeStamp = timeStamp;
    }

    public double getTime() {
        return time;
    }

    public double getPos() {
        return pos;
    }

    public double getVel() {
        return vel;
    }

    public double getAcc() {
        return acc;
    }

    public double getHeading() {
        return heading;
    }

    public double getTimeStamp() {
        return timeStamp;
    }

}