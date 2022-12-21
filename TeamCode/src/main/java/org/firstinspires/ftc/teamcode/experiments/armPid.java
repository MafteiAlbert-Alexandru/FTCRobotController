//package org.firstinspires.ftc.teamcode.experiments;
//import java.util.Date;
//import java.util.Calendar;
//import java.lang.*;
//
//public class armPid {
//    public class Coefficients{
//        double kp;//proportional
//        double ki;//integral
//        double kd;//derivative
//        double ka;//angular
//
//        Coefficients(){}
//        Coefficients(Coefficients coefficients){
//            this.kp = coefficients.kp;
//            this.ki = coefficients.ki;
//            this.kd = coefficients.kd;
//            this.ka = coefficients.ka;
//        }
//    }
//
//    private Coefficients coefficients;
//    private double tolerance;
//    private double setPoint;
//
//    private class FrameResources{
//        public double currentPoint;
//        public long time1_ms;
//        public long time2_ms;
//        public long deltaTime_ms;
//        public double error1;
//        public double error2;
//        public double integral;
//    }
//    private FrameResources frame;
//
//
//    armPid(Coefficients coefficients){
//        this.coefficients = new Coefficients(coefficients);
//    }
//
//    void setCoefficients(){
//        this.coefficients = new Coefficients(coefficients);
//    }
//    void setTolerance(double tolerance){
//        this.tolerance = tolerance;
//    }
//    Coefficients getCoefficients(){
//        return this.coefficients;
//    }
//    void setSetPoint(double setPoint){
//        this.setPoint = setPoint;
//    }
//    double calculate(double currentPoint, double setPoint){
//        if(setPoint - currentPoint < tolerance)return 0;
//
//        double pida;
//
//        frame.error2 = setPoint - currentPoint;
//        frame.currentPoint = currentPoint;
//        frame.time2_ms = System.currentTimeMillis();
//        frame.deltaTime_ms = frame.time2_ms - frame.time1_ms;
//        frame.integral += frame.deltaTime_ms * frame.error2;
//
//        double p = frame.error2;
//        double i = frame.integral;
//        double d = (frame.error2 - frame.error1) / frame.deltaTime_ms;
//        double a = currentPoint * Math.cos(currentPoint);
//
//        pida = p*coefficients.kp + i*coefficients.ki + d*coefficients.kd + a*coefficients.ka;
//
//        return pida;
//    }
//    double calculate(int currentPoint_ticks, int setPoint_ticks, int ticksPer360degrees){
//        double currentPoint_degrees = (currentPoint_ticks / ticksPer360degrees) * 360;
//        double setPoint_degrees =(setPoint_ticks / ticksPer360degrees) * 360;
//
//        return calculate(currentPoint_degrees, setPoint_degrees);
//    }
//}
