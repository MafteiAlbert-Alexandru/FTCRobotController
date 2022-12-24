package org.firstinspires.ftc.teamcode.experiments;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;

public class BasicPIDFixed implements FeedbackController {

    PIDCoefficients coefficients;

    protected boolean hasRun = false;

    protected Timer timer = new Timer();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    public BasicPIDFixed(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    /**
     * calculate PID output
     * @param reference the target position
     * @param state current system state
     * @return PID output
     */
    @Override
    public double calculate(double reference, double state) {
        double dt = getDT();
        double error = calculateError(reference, state);
        integrate(error, dt);
        double derivative = calculateDerivative(error,dt);
        previousError = error;
        return error * coefficients.Kp
                + integralSum * coefficients.Ki
                + derivative * coefficients.Kd;
    }

    /**
     * get the time constant
     * @return time constant
     */
    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.currentTime();
        timer.reset();
        return dt;
    }

    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    protected void integrate(double error, double dt) {
        integralSum += (error) * dt*1/1000.0;
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }

}
