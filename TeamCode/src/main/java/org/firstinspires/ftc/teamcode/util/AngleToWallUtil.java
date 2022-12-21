//package org.firstinspires.ftc.teamcode.util;
//
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.filters.SimpleKalmanFilter;
//
//public class AngleToWallUtil {
//    private static final double distanceSensorVariance=0.01;
//    private final double distanceBetweenSensors;
//    private final DistanceSensor frontSensor, backSensor;
//    private final SimpleKalmanFilter frontKalmanFilter, backKalmanFilter;
//
//    public AngleToWallUtil(DistanceSensor frontSensor, DistanceSensor backSensor, double distanceBetweenSensors)
//    {
//        this.frontSensor=frontSensor;
//        this.backSensor=backSensor;
//        this.distanceBetweenSensors=distanceBetweenSensors;
//        frontKalmanFilter=new SimpleKalmanFilter(distanceSensorVariance);
//        backKalmanFilter=new SimpleKalmanFilter(distanceSensorVariance);
//    }
//
//    /**
//     * Updates the sensor measurements
//     */
//    public void update()
//    {
//        frontKalmanFilter.update(frontSensor.getDistance(DistanceUnit.MM));
//        backKalmanFilter.update(backSensor.getDistance(DistanceUnit.MM));
//    }
//
//    /**
//     * Returns distance to wall
//     * @return Angle to the wall (in radians)
//     */
//    public double angle()
//    {
//            return Math.atan(Math.abs(frontKalmanFilter.value()-backKalmanFilter.value())/distanceBetweenSensors);
//    }
//}
