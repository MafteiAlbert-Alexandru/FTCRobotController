package org.firstinspires.ftc.teamcode.util;

public class SmartMath {

    public static <T extends Comparable<T>> T Clamp(T val, T min, T max) {
        if (val.compareTo(min) < 0) return min;
        if (val.compareTo(max) > 0) return max;
        return val;
    }

    public static boolean ToleranceError(double x, double y,double tolerance){


        return (x>y-tolerance && x<y+tolerance);
    }

    public static double TicksToDistance(int ticks, double revRatio, int angle){
        return Math.round(ticks*revRatio*Math.cos(angle)*100)/100;
    }

    public static double TicksToDistance(int ticks, double revRatio){
        return TicksToDistance(ticks, revRatio, 0);
    }

    public static double DistanceToTicks(int distance, double revRatio, int angle){
        return Math.round(distance/revRatio/Math.cos(angle)*100)/100;
    }

    public static double DistanceToTicks(int distance, double revRatio){
        return DistanceToTicks(distance, revRatio, 0);
    }

}
