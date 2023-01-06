package org.firstinspires.ftc.teamcode.experiments;

public class PIDExCoefficients extends PIDCoefficients {

    public double maxIntegralSum;
    public double integralGain;

    public PIDExCoefficients(double kP, double kI, double kD, double maxIntegralSum, double integralGain) {
        super(kP, kI, kD);
        this.maxIntegralSum=maxIntegralSum;
        this.integralGain=integralGain;
    }
}
