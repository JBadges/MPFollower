package follower;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import auto.AutoPath;
import auto.TimeRangedCommand;
import edu.wpi.first.wpilibj.Notifier;
import pid.PID;
import pid.TrajPID;
import trajectory.TrajectoryPoint;

public class Follower {
  protected Consumer<Double> setLeftPercent;
  protected Consumer<Double> setRightPercent;
  protected Supplier<Double> getLeftDistance;
  protected Supplier<Double> getRightDistance;
  protected Supplier<Double> getHeading;
  
  private Notifier thread;
  protected AutoPath autoPath;
  protected TrajPID trajPid;
  protected PID headingPid;
  
  protected String debugFilePath = "/home/lvuser/trajDebug.csv";
  
  public Follower(Consumer<Double> setLeftPercent, Consumer<Double> setRightPercent, Supplier<Double> getLeftDistance,
      Supplier<Double> getRightDistance, Supplier<Double> getHeading, TrajPID trajPid, PID headingPid) {
    this.setLeftPercent = setLeftPercent;
    this.setRightPercent = setRightPercent;
    this.getLeftDistance = getLeftDistance;
    this.getRightDistance = getRightDistance;
    this.getHeading = getHeading;
    this.trajPid = trajPid;
    this.headingPid = headingPid;
  }
  
  public void setAutoPath(AutoPath autoPath) {
	  this.autoPath = autoPath;
  }
  
  public void start() {
    TrajectoryUpdater tu = new TrajectoryUpdater(this);
    tu.reset();
    thread = new Notifier(tu);
    thread.startPeriodic(tu.getTimeStep() / 2.0);
  }
  
  public void stop() {
    thread.stop();
  }
  
}

class TrajectoryUpdater implements Runnable {
  
  private long time;
  private long systemTimeAtStart;
  private double timeStep;
  private List<TrajectoryPoint> leftTraj;
  private List<TrajectoryPoint> rightTraj;
  private TimeRangedCommand[] commands;
  
  private FileWriter fw;
  
  private Follower follower;
  
  public TrajectoryUpdater(Follower follower) {
    leftTraj = new ArrayList<>();
    rightTraj = new ArrayList<>();
    this.follower = follower;
  }
  
  public void reset() {
    systemTimeAtStart = System.currentTimeMillis();
    time = 0;
    clearAndLoad();
    getTimeStep();
    new File(follower.debugFilePath).delete();
    try {
      fw = new FileWriter(follower.debugFilePath);
      fw.write("Left Cur Pos, Left Goal Pos, Left Goal Error, Left Goal Vel, Left Goal Acc, Right Cur Pos, Right Goal Pos, Right Goal Error, Right Goal Vel, Right Goal Acc");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
  
  private void clearAndLoad() {
    leftTraj = Arrays.asList(follower.autoPath.getLeftTrajectory());
    rightTraj = Arrays.asList(follower.autoPath.getRightTrajectory());
    commands = follower.autoPath.getCommands();
  }
  
  @Override
  public void run() {   
    time = System.currentTimeMillis() - systemTimeAtStart;
    
    for(TimeRangedCommand c : commands) {
    	c.tryAtTime(time);
    }
    
    int index = (int) (time / timeStep);
    
    TrajectoryPoint leftTP = leftTraj.get(index >= leftTraj.size() ? leftTraj.size()-1 : index);
    TrajectoryPoint rightTP = rightTraj.get(index >= rightTraj.size() ? rightTraj.size()-1 : index);

    double leftOut = follower.trajPid.getValue(follower.getLeftDistance.get(), leftTP.getPosition(), leftTP.getVelocity(),
        leftTP.getAcceleration());
    double rightOut = follower.trajPid.getValue(follower.getRightDistance.get(), rightTP.getPosition(), rightTP.getVelocity(),
        rightTP.getAcceleration());
    double headingOffset = follower.headingPid.getValue(follower.getHeading.get(), leftTP.getHeading());
    rightOut += headingOffset;
    leftOut -= headingOffset;
    
    follower.setLeftPercent.accept(leftOut);
    follower.setRightPercent.accept(rightOut);
    
    //DEBUG
    try {
      fw.write(follower.getLeftDistance.get() + ", ");
      fw.write(leftTP.getPosition() + ", ");
      fw.write(leftTP.getPosition() - follower.getLeftDistance.get() + ", ");
      fw.write(leftTP.getVelocity() + ", ");
      fw.write(leftTP.getAcceleration() + ", ");
      fw.write(follower.getRightDistance.get() + ", ");
      fw.write(rightTP.getPosition() + ", ");
      fw.write(rightTP.getPosition() - follower.getRightDistance.get() + ", ");
      fw.write(rightTP.getVelocity() + ", ");
      fw.write(rightTP.getAcceleration() + ",\n");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
  
  public double getTimeStep() {
    timeStep = leftTraj.get(0).getTimeStep();
    return timeStep;
  }
  
}