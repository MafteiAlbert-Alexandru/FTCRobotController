package org.firstinspires.ftc.teamcode.experiments;

public class PIDController {
    private PIDCoefficients coefficients;
    protected double integral=0;
    protected double lastError=0;
    protected long lastTime=-1;
    public PIDController(PIDCoefficients coefficients)
    {
        this.coefficients = coefficients;
    }
    public void reset()
    {
        integral=0;
        lastError=0;
        lastTime=-1;
    }
    public double update(double target, double value)
    {

        double error = target - value;
        if(lastTime==-1)
        {
            lastTime=System.currentTimeMillis();
            lastError=error;
            return 0;
        }else {
            long time = System.currentTimeMillis();
            double deltaTime = (time-lastTime)/1000.0;
            lastTime=time;
            integral+=error*deltaTime;
            double derivative = (error-lastError)*deltaTime;
            lastError=error;
            return coefficients.kP*error*deltaTime + coefficients.kI*integral + coefficients.kD*derivative;
        }

    }
}
