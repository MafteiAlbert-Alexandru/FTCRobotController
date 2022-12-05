package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.filters.SimpleKalmanFilter;

public class DistanceToWallUtil {

    private static final double distanceSensorVariance=0.01;
    private final DistanceSensor leftSensor, rightSensor;
    private final SimpleKalmanFilter leftKalmanFilter, rightKalmanFilter;

    public DistanceToWallUtil(DistanceSensor leftSensor, DistanceSensor rightSensor)
    {
        this.leftSensor=leftSensor;
        this.rightSensor=rightSensor;
        leftKalmanFilter=new SimpleKalmanFilter(distanceSensorVariance);
        rightKalmanFilter=new SimpleKalmanFilter(distanceSensorVariance);
    }

    /**
     * Updates the sensor measurements
     */
    public void update()
    {
        leftKalmanFilter.update(leftSensor.getDistance(DistanceUnit.MM));
        rightKalmanFilter.update(rightSensor.getDistance(DistanceUnit.MM));
    }

    /**
     * Returns distance to wall
     * @return Distance to the wall (left = negative)
     */
    public double distance()
    {
        if(rightKalmanFilter.value()>leftKalmanFilter.value())
        {
            return rightKalmanFilter.value();
        }else return -leftKalmanFilter.value();
    }




}
