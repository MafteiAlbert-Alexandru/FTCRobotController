package org.firstinspires.ftc.teamcode.experiments;


import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

/**
 * extended PID controller, implements anti wind up and derivative smoothing features.
 */
public class PIDExFixed extends BasicPIDFixed {
    protected PIDCoefficientsEx basedCoefficients;
    public PIDExFixed(PIDCoefficientsEx coefficients) {
        super(coefficients);
        this.basedCoefficients = coefficients;
    }

    @Override
    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }

    /**
     * Integral with anti windup methods
     * @param error the current error of our system
     * @param dt the time constant.
     */
    @Override
    protected void integrate(double error, double dt) {
        if (crossOverDetected(error,previousError)) integralSum = 0;
        if (Math.abs(derivative) > basedCoefficients.stabilityThreshold) return;
        integralSum += ((error + previousError) / 2) * dt;
        if (Math.abs(integralSum) > basedCoefficients.maximumIntegralSum) {
            integralSum = Math.signum(integralSum) * basedCoefficients.maximumIntegralSum;
        }
    }

    protected boolean crossOverDetected(double error, double prev) {
        if (error > 0 && prev < 0) return true;
        return error < 0 && prev > 0;
    }

}
