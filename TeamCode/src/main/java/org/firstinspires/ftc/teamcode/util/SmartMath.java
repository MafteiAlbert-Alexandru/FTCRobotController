package org.firstinspires.ftc.teamcode.util;

public class SmartMath{

    public static <T extends Comparable<T>> T Clamp(T val, T min, T max) {
        if (val.compareTo(min) < 0) return min;
        if (val.compareTo(max) > 0) return max;
        return val;
    }

    public static boolean ToleranceError(double x, double y,double tolerance){
        return (x>y-tolerance && x<y+tolerance);
    }
}
