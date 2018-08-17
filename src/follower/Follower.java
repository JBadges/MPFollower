package follower;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import log.Logger;
import path.TrajectoryPoint;
import pid.PID;
import pid.TrajPID;

public class Follower {
    protected Consumer<Double> setLeftPercent;
    protected Consumer<Double> setRightPercent;
    protected Supplier<Double> getLeftDistance;
    protected Supplier<Double> getRightDistance;
    protected Supplier<Double> getHeading;

    private Notifier thread;
    protected String leftPath;
    protected String rightPath;
    protected TrajPID trajPid;
    protected PID headingPid;

    public Follower(Consumer<Double> setLeftPercent, Consumer<Double> setRightPercent, Supplier<Double> getLeftDistance, Supplier<Double> getRightDistance,
            Supplier<Double> getHeading, TrajPID trajPid, PID headingPid) {
        this.setLeftPercent = setLeftPercent;
        this.setRightPercent = setRightPercent;
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

    public void start() {
        TrajectoryUpdater tu = new TrajectoryUpdater(this);
        tu.reset();
        thread = new Notifier(tu);
        thread.startPeriodic(tu.getTimeStep()/1000.0);
    }

    public void stop() {
        thread.stop();
    }

}

class TrajectoryUpdater implements Runnable {

    private long time;
    private long systemTimeAtStart;
    private long timeStep;
    private List<TrajectoryPoint> leftTraj;
    private List<TrajectoryPoint> rightTraj;
    
    private Follower follower;

    public TrajectoryUpdater(Follower follower) {
        this.follower = follower;
    }

    public void reset() {
        systemTimeAtStart = System.currentTimeMillis();
        time = 0;
        clearAndLoadTrajectories();
        getTimeStep();
    }

    private void clearAndLoadTrajectories() {
        leftTraj.clear();
        rightTraj.clear();
        
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(follower.leftPath));
        } catch (FileNotFoundException e) {
            Logger.write("ERROR: ", "Left path file is invalid");
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
            Logger.write("ERROR: ", "Invalid number while reading the left trajectory");
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            reader = new BufferedReader(new FileReader(follower.rightPath));
        } catch (FileNotFoundException e) {
            Logger.write("ERROR: ", "Right path file is invalid");
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
            Logger.write("ERROR: ", "Invalid number while reading the right trajectory");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void run() {
        time = System.currentTimeMillis() - systemTimeAtStart;
        long index = time / timeStep;
        
        TrajectoryPoint leftTP = leftTraj.get((int) index);
        TrajectoryPoint rightTP = rightTraj.get((int) index);
        
        double leftOut = follower.trajPid.getValue(follower.getLeftDistance.get(), leftTP.getPos(), leftTP.getVel(), leftTP.getAcc());
        double rightOut = follower.trajPid.getValue(follower.getRightDistance.get(), rightTP.getPos(), rightTP.getVel(), rightTP.getAcc());
        double headingOffset = follower.headingPid.getValue(follower.getHeading.get(), leftTP.getHeading());
        rightOut += headingOffset;
        leftOut -= headingOffset;
        
        follower.setLeftPercent.accept(leftOut);
        follower.setRightPercent.accept(rightOut);
    }
    
    public long getTimeStep() {
        timeStep = (long) leftTraj.get(0).getTimeStamp();
        return timeStep;
    }

}