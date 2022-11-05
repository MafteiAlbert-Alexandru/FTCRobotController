package org.firstinspires.ftc.teamcode.junctionCalibration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class junctionAdjuster {
    private OpenCvCamera camera;

    public class CameraData{
        public int resolution_x;
        public int resolution_y;
        public double FOV_x;
        public double FOV_y;
    }

    public class JunctionData{
        public double diameter;
    }

    public class MecanumResources{
        //Aici vin motoarele de la roti
    }


    private class Resources{
        public double integral;

        public double ang_0;
        public double ang_1;

        public double distToJunction;
        public double camField;
        public double junctionField;
    }


    private CameraData Cam;
    private JunctionData junction;
    private MecanumResources mecanum;
    private Resources frame;

    private junctionAdjusterPipeline.Results results_0;
    private junctionAdjusterPipeline.Results results_1;

    public double kp;
    public double ki;
    public double kd;

    private junctionAdjusterPipeline pipeline;

    public junctionAdjuster(OpenCvCamera camera_, CameraData cam_, JunctionData junction_, MecanumResources mecanum_){
        this.camera = camera_;

        this.Cam = cam_;
        this.junction = junction_;
        this.mecanum = mecanum_;

        kp = 1;     //
        ki = 1;     // Trebuiesc ajustate
        kd = 1;     //

    }

    public int update(double maxSpeed){
        junctionAdjusterPipeline.Results results = junctionAdjusterPipeline.getLatestResults();

        if (results.wasAccesed == false) { // prima accesare a datelor de pe camera dupa o calculare asincron a junctionului
            results.wasAccesed = true;

            results_0 = results_1;
            results_1 = results;

            frame.ang_0 = frame.ang_1;
            frame.ang_1 = Math.toRadians(((double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1) * Cam.FOV_x); // unghiul dintre vectorul de orientare a robotului si vectorul robot-junction

            frame.distToJunction = junction.diameter / Math.tan(frame.ang_1);   //
            frame.camField = 2 * (Math.tan(Cam.FOV_x) * frame.distToJunction);  // nu sunt folosite dar ar fi bine sa le pastram
            frame.junctionField = Math.tan(frame.ang_1) * frame.distToJunction; //

            frame.integral += frame.ang_1 * (results_1.time - results_0.time);
        }

        double Error_1 = frame.ang_1;
        double Error_0 = frame.ang_0;
        double deltaError = Error_1 - Error_0;
        long frameTime = System.nanoTime() - results.time;
        long deltaTime = results.time - results_1.time;

        if(deltaTime == 0)deltaTime = 1;
        if(frameTime == 0)frameTime = 1;

        if(Error_1 != 0){

            double p = Error_1 * kp;                          // p - proportional gain
            double i = frame.integral * ki;                   // i - integral gain
            double d = (deltaError / deltaTime) * kd;         // d - derivative gain

            double PID = p + i + d;
            if(PID > 1)PID = 1;

            double direction = Math.copySign(1, Error_1);
            double power = PID * maxSpeed;

            turn(direction, power);

            return 0;
        }
        return 1;
    }


    private void turn(double direction, double power){//       directie: -1 = stanga, 1 = dreapta

    }

}
