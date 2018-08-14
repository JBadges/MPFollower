package pid;

public class TrajPID extends PID {

    private double Kv;
    private double Ka;

    public TrajPID(double kp, double ki, double kd, double kv, double ka) {
        this(kp, ki, kd, kv, ka, -1e10, 1e10);
    }
    
    public TrajPID(double kp, double ki, double kd, double kv, double ka, double min, double max) {
        super(kp, ki, kd, min, max);
        Kv = kv;
        Ka = ka;
    }

    public double getValue(double curPos, double goalPos, double goalVel, double goalAcc) {
        double out = super.getValue(curPos, goalPos);
        out += goalVel * Kv + goalAcc * Ka;
        out = Math.max(Math.min(getMax(), out), getMin());
        return out;
    }
    
}
