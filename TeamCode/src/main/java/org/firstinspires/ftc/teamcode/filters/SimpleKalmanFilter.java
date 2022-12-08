package org.firstinspires.ftc.teamcode.filters;



public class SimpleKalmanFilter extends Filter<Double, Double> {



    private double estimatedError=1;
    private double estimate;
    private final double measurementVariance;

    /**
     * Simple Kalman filter for one variable
     * @param measurementVariance variance of data(lower = faster reaction speeds, but less precise)
     */
    public SimpleKalmanFilter(double measurementVariance)
    {
        this.measurementVariance=measurementVariance;
    }

    @Override
    public void update(Double data)
    {
        final double kalmanGain = estimatedError / (estimatedError + measurementVariance);
        estimate+=kalmanGain*(data - estimate);
        estimatedError*=(1-kalmanGain);
    }

    @Override
    public Double value() {
        return estimate;
    }

}
