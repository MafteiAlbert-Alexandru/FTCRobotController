package org.firstinspires.ftc.teamcode.junctionCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class junctionAdjuster {
    private OpenCvCamera camera;

    public class CameraData{
        public int resolution_x;
        public int resolution_y;
        public double FOV_x;
    }

    public class JunctionData{
        public double diameter;
    }

    public class Vec2{
        public double x;
        public double y;
    }


    /*private class Resources{
        public double integral;

        public double ang_0;
        public double ang_1;

        public double distToJunction;
        public double camField;
        public double junctionField;
    }*/


    private CameraData Cam;
    private JunctionData junction;
    //private Resources frame = new Resources();

    /*private junctionAdjusterPipeline.Results results_0 = new junctionAdjusterPipeline.Results();
    private junctionAdjusterPipeline.Results results_1 = new junctionAdjusterPipeline.Results();

    public double kp;
    public double ki;
    public double kd;*/

    private junctionAdjusterPipeline pipeline;

    Telemetry telemetry;

    public junctionAdjuster(OpenCvCamera camera_, float FOV_x, int resolution_x, int resolution_y, double diameter, Telemetry telemetry_){
        camera = camera_;

        Cam = new CameraData();
        junction = new JunctionData();

        Cam.FOV_x = FOV_x;
        Cam.resolution_x = resolution_x;
        Cam.resolution_y = resolution_y;

        junction.diameter = diameter;

        pipeline = new junctionAdjusterPipeline();
        camera.setPipeline(pipeline);

        telemetry = telemetry_;

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(  Cam.resolution_x, Cam.resolution_y, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

                // This will be called if the camera could not be opened

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

    }

    public Vec2 relativeJunctionPosition(){ // cm
        Vec2 position = new Vec2();

        double angle;
        double distToJunction;

        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        angle = Math.toRadians(Math.abs((double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1) * Cam.FOV_x); // unghiul dintre vectorul de orientare a robotului si vectorul robot-junction
        distToJunction = junction.diameter / Math.tan(angle);

        position.x = Math.copySign(Math.tan(angle) * distToJunction, (double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1);
        position.y = distToJunction;

        return position;
    }

    /*public int update(double maxSpeed){
        junctionAdjusterPipeline.Results results = pipeline.getLatestResults();

        telemetry.addData("wasAccesed ", results.wasAccesed);

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

            telemetry.addData("distance ", frame.distToJunction);
        }

        telemetry.update();

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
    }*/
}
