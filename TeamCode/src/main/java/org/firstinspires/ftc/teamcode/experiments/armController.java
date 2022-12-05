package org.firstinspires.ftc.teamcode.experiments;

/*import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Date;
import java.util.Calendar;
import java.lang.*;

public class armController {
    private Motor motor;
    public boolean active = false;

    //  A... - angular
    //   C... - change
    //


    private class Physics{
        public double mass;
        public double armLength;
        public double inertia;
        public final double g = 9.81;
        public double maxVelocity;
        public double targetVelocity;
        public double torque;
        public double Ctime;
        public double CAng;
        public double CdeltaAVel;
        public double CdeltaTime;
    }
    Physics physics;

    private class PID{
        public double[] Error = {0,0};
        public double Integral = 0;
        public double Tolerance = 0;
    }
    PID controller;

    private double targetAngle;
    private double startAngle;
    private double lastAngle;

    private long startTime;
    private long sampleTime;
    private long time;

    public armController(Motor motor, double armLength, double mass, double torque)
    {
        this.motor = motor;
        physics.armLength = armLength;
        physics.mass = mass;
        physics.inertia = mass*armLength*armLength*(1/3);
        physics.maxVelocity = (2*Math.PI*armLength)/(1/motor.getMaxRPM());
        physics.torque = torque;
    }

    public void Rotate(double targetAngle, int samplesPerSecond, double velocity)
    {
        active = true;
        this.targetAngle = targetAngle;
        sampleTime = (long)(1000000000/samplesPerSecond);
        this.startAngle = getRad((int)motor.getCurrentPosition());
        lastAngle = startAngle;
        startTime = System.nanoTime();
        time = 0;
        motor.setTargetPosition(getTicks(targetAngle));

        physics.targetVelocity = (velocity * Math.copySign(1, targetAngle - startAngle)) * physics.maxVelocity;
        physics.CdeltaTime = ((physics.maxVelocity/physics.armLength)*(physics.maxVelocity/physics.armLength))/physics.torque;
        physics.CAng = targetAngle-(physics.inertia* physics.CdeltaTime);
        physics.CdeltaAVel = physics.targetVelocity/physics.armLength;
        physics.Ctime = Math.abs(physics.CAng-startAngle)/(physics.targetVelocity * physics.armLength);
    }

    private void sample(long deltaTime)
    {
        double kp;
        double ki;
        double kd;

        double currentAng = getRad((int)motor.getCurrentPosition());
        double currentVel = motor.getCorrectedVelocity();//estimated ((currentRad-lastRad)*armLength)/deltaTime


        double idealVel = PhysicalVelocity(currentAng, currentVel, deltaTime);


        lastAngle = currentAng;

        controller.Error[0] = controller.Error[1];
        controller.Error[1] = idealVel - currentVel;

        if(Math.abs(controller.Error[1]) < 0.1)controller.Integral = 0;   // The integral is estimated
        controller.Integral += (controller.Error[1]-controller.Error[0])*deltaTime;  //  More samples per second = more accurate Integral

        kp = controller.Error[1];
        ki = controller.Integral;
        kd = (controller.Error[1]-controller.Error[0])/deltaTime;

        if(Math.abs(kp+ki+kd) > controller.Tolerance)
            motor.setVeloCoefficients(kp, ki, kd);
    }

    public void update()
    {
        if(active)
        {
            if(motor.atTargetPosition())
            {
                active = false;
                motor.set(0);
                return;
            }
            if(System.nanoTime() - startTime >= sampleTime)
            {
                time += System.nanoTime() - startTime;
                sample(System.nanoTime() - startTime);
                startTime = System.nanoTime();
            }
        }
    }
    private double PhysicalVelocity(double currentAng, double currentVel, long deltaTime){
        if(Math.copySign(1, getRad(motor.getCurrentPosition()) - physics.CAng) != Math.copySign(1, getRad(motor.getCurrentPosition()) - targetAngle)){ // check if the motor should decelerate
            double deltaAVel = physics.CdeltaAVel * (time - (long)(physics.Ctime * 1000000000)) * (1 / physics.CdeltaTime);
            return physics.targetVelocity+(deltaAVel * physics.armLength);
        }
        return physics.targetVelocity;
    }
    public void setTolerance(double tolerance){
        controller.Tolerance = tolerance;
    }

    private int getTicks(double radPosition){
        if(motor.getInverted()){
            return (int)(-radPosition * motor.getCPR()*2*Math.PI);
        }else{
            return (int)(radPosition * motor.getCPR()*2*Math.PI);
        }
    }
    private double getRad(int encoderPosition)
    {
        if(motor.getInverted())
        {
            return -encoderPosition / motor.getCPR()*2*Math.PI;
        }else{
            return encoderPosition / motor.getCPR()*2*Math.PI;
        }
    }
}*/
