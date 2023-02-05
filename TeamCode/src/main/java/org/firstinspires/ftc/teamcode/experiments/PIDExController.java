package org.firstinspires.ftc.teamcode.experiments;

import java.util.Random;

public class PIDExController extends PIDController{
    private PIDExCoefficients coefficients;
    private Random delayRandom= new Random();
    public PIDExController(PIDExCoefficients coefficients) {
        super(coefficients);
        this.coefficients=coefficients;
    }
    public double lastDT()
    {
        return (System.currentTimeMillis()-lastTime)/1000.0;
    }
    @Override
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
            integral+=error*coefficients.integralGain*deltaTime;
            if(Math.abs(integral)> coefficients.maxIntegralSum)
            {
                integral=Math.signum(integral)* coefficients.maxIntegralSum;
            }
            if(Math.signum(lastError)!=Math.signum(error))
            {
                integral=0;
            }
            double derivative = (error-lastError)*deltaTime;
            lastError=error;
            try {
                Thread.sleep((long) (delayRandom.nextDouble()*25));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return coefficients.kP*error*deltaTime + coefficients.kI*integral + coefficients.kD*derivative;
        }
    }
}
