package pid;

public class PID {

    private double Kp;
    private double Ki;
    private double Kd;
    private double min;
    private double max;

    private double lastError;
    private double totError;

    public PID(double kp, double ki, double kd) {
            this(kp, ki, kd, -1e10, 1e10);
        }

    public PID(double kp, double ki, double kd, double min, double max) {
            Kp = kp;
            Ki = ki;
            Kd = kd;        
            this.min = min;
            this.max = max;
        }

    public void reset() {
        lastError = 0;
        totError = 0;
    }

    public double getValue(double cur, double goal) {
        double p = goal - cur;
        double i = totError;
        double d = lastError;
        lastError = p;
        totError += p;

        double out = p * Kp + i * Ki + d * Kd;
        out = Math.max(Math.min(max, out), min);
        return out;
    }
    
    public double getMin() {
        return min;
    }
    
    public double getMax() {
        return max;
    }
    

}
