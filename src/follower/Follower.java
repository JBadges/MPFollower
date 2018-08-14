package follower;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import pid.PID;
import pid.TrajPID;

public class Follower {
    private Consumer<Double> setLeftPercent;
    private Consumer<Double> setRightPercent;
    private Supplier<Double> getLeftVelocity;
    private Supplier<Double> getRightVelocity;
    private Supplier<Double> getLeftDistance;
    private Supplier<Double> getRightDistance;
    private Supplier<Double> getHeading;

    private Notifier thread;
    private String leftPath;
    private String rightPath;
    private TrajPID trajPid;
    private PID headingPid;

    public Follower(Consumer<Double> setLeftPercent, Consumer<Double> setRightPercent, Supplier<Double> getLeftVelocity,
            Supplier<Double> getRightVelocity, Supplier<Double> getLeftDistance, Supplier<Double> getRightDistance,
            Supplier<Double> getHeading, TrajPID trajPid, PID headingPid) {
        this.setLeftPercent = setLeftPercent;
        this.setRightPercent = setRightPercent;
        this.getLeftVelocity = getLeftVelocity;
        this.getRightVelocity = getRightVelocity;
        this.getLeftDistance = getLeftDistance;
        this.getRightDistance = getRightDistance;
        this.getHeading = getHeading;
        this.trajPid = trajPid;
        this.headingPid = headingPid;
    }

    public void setLeftTrajectory(String filePath) {
        leftPath = filePath;
    }

    public void setRightTrajectory(String filePath) {
        rightPath = filePath;
    }

    public void start(double period) {
        TrajectoryUpdater tu = new TrajectoryUpdater(setLeftPercent, setRightPercent, getLeftVelocity, getRightVelocity, getLeftDistance, getRightDistance, getHeading, leftPath, rightPath, trajPid, headingPid);
        tu.reset();
        thread = new Notifier(tu);
        thread.startPeriodic(period);
    }

    public void stop() {
        thread.stop();
    }

}

class TrajectoryUpdater implements Runnable {

    private long time;
    private long systemTimeAtStart;
    private long timeStamp;
    private String leftPath;
    private String rightPath;
    private List<TrajectoryPoint> leftTraj;
    private List<TrajectoryPoint> rightTraj;
    private Consumer<Double> setLeftPercent;
    private Consumer<Double> setRightPercent;
    private Supplier<Double> getLeftVelocity;
    private Supplier<Double> getRightVelocity;
    private Supplier<Double> getLeftDistance;
    private Supplier<Double> getRightDistance;
    private Supplier<Double> getHeading;
    private TrajPID trajPid;
    private PID headingPid;


    public TrajectoryUpdater(Consumer<Double> setLeftPercent, Consumer<Double> setRightPercent,
            Supplier<Double> getLeftVelocity, Supplier<Double> getRightVelocity, Supplier<Double> getLeftDistance,
            Supplier<Double> getRightDistance, Supplier<Double> getHeading, String leftPath, String rightPath, TrajPID trajPid, PID headingPid) {
        this.setLeftPercent = setLeftPercent;
        this.setRightPercent = setRightPercent;
        this.getLeftVelocity = getLeftVelocity;
        this.getRightVelocity = getRightVelocity;
        this.getLeftDistance = getLeftDistance;
        this.getRightDistance = getRightDistance;
        this.getHeading = getHeading;
        this.leftPath = leftPath;
        this.rightPath = rightPath;
        this.trajPid = trajPid;
        this.headingPid = headingPid;
    }

    public void reset() {
        systemTimeAtStart = System.currentTimeMillis();
        time = 0;
        leftTraj.clear();
        rightTraj.clear();

        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(leftPath));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        String line;
        try {
            while ((line = reader.readLine()) != null) {
                String[] s = line.split(",");
                TrajectoryPoint tp = new TrajectoryPoint(Double.parseDouble(s[0]), Double.parseDouble(s[1]),
                        Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]),
                        Double.parseDouble(s[5]));
                leftTraj.add(tp);
            }
        } catch (NumberFormatException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            reader = new BufferedReader(new FileReader(rightPath));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        try {
            while ((line = reader.readLine()) != null) {
                String[] s = line.split(",");
                TrajectoryPoint tp = new TrajectoryPoint(Double.parseDouble(s[0]), Double.parseDouble(s[1]),
                        Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]),
                        Double.parseDouble(s[5]));
                rightTraj.add(tp);
            }
        } catch (NumberFormatException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        time = System.currentTimeMillis() - systemTimeAtStart;
        long index = time / timeStamp;
        
        TrajectoryPoint leftTP = leftTraj.get((int) index);
        TrajectoryPoint rightTP = rightTraj.get((int) index);
        
        double leftOut = trajPid.getValue(getLeftDistance.get(), leftTP.getPos(), leftTP.getVel(), leftTP.getAcc());
        double rightOut = trajPid.getValue(getRightDistance.get(), rightTP.getPos(), rightTP.getVel(), rightTP.getAcc());
        double headingOffset = headingPid.getValue(getHeading.get(), leftTP.getHeading());
        rightOut += headingOffset;
        leftOut += headingOffset;
        
        setLeftPercent.accept(leftOut);
        setRightPercent.accept(rightOut);
    }

}

class TrajectoryPoint {
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