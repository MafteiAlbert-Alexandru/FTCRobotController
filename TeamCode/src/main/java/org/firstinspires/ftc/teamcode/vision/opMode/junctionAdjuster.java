//package org.firstinspires.ftc.teamcode.vision.opMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//public class junctionAdjuster {
//    private int cameraMonitorViewId;
//    private OpenCvCamera camera;
//    private WebcamName webcamName;
//
//    public class CameraData{
//        public int resolution_x;
//        public int resolution_y;
//        public double FOV_x;
//        public double FOV_y;
//    }
//
//    public class JunctionData{
//        public double diameter;
//    }
//
//    public class MecanumResources{
//        //Aici vin motoarele de la roti
//    }
//
//
//    private class Resources{
//        public double ang;
//        public double distToJunction;
//        public double camField;
//        public double junctionField;
//    }
//
//
//    private CameraData Cam;
//    private JunctionData junction;
//    private MecanumResources mecanum;
//    private Resources frame;
//
//    public double kp;
//    public double ki;
//    public double kd;
//
//    private junctionAdjusterPipeline pipeline;
//
//    public junctionAdjuster(OpenCvCamera camera_, WebcamName webcamName_, int cameraMonitorViewId_, CameraData cam_, JunctionData junction_, MecanumResources mecanum_){
//        this.camera = camera_;
//        this.webcamName = webcamName_;
//        this.cameraMonitorViewId = cameraMonitorViewId_;
//
//        this.Cam = cam_;
//        this.junction = junction_;
//        this.mecanum = mecanum_;
//
//        kp = 1;     //
//        ki = 1;     // Trebuiesc ajustate
//        kd = 1;     //
//
//        camera.setPipeline(pipeline);
//
//        OpenCamDevice();
//    }
//
//    public int update(double maxSpeed){
//        pipeline.Results results = pipeline.getLatestResults();
//
//        if (results.wasAccesed == false) { // prima accesare a datelor de pe camera dupa o calculare asincron a junctionului
//            results.wasAccesed = true;
//            frame.ang = Math.toRadians(((double)(results.junction_x1 + results.junction_x2) / (double)(Cam.resolution_x * 2) - 1) * Cam.FOV_x); // unghiul dintre vectorul de orientare a robotului si vectorul robot-junction
//
//            frame.distToJunction = junction.diameter / Math.tan(frame.ang);   //
//            frame.camField = 2 * (Math.tan(Cam.FOV_x) * frame.distToJunction);                              // nu sunt folosite dar ar fi bine sa le pastram
//            frame.junctionField = Math.tan(frame.ang) * frame.distToJunction;                               //
//        }
//
//        double Error = frame.ang;
//        long deltaTime = System.nanoTime() - results.time;
//        if(deltaTime == 0)deltaTime = 1;
//
//        if(Error != 0){
//            double p = Error * kp;                          // p - ajusteaza in functie de ultimele date de pe camera
//            double i = Error * ki * 1/((double)deltaTime);  // i - tine cont si de timp
//            double d = 0;                                   // d - idk
//
//            double PID = p + i + d;
//            if(PID > 1)PID = 1;
//
//            double direction = Math.copySign(1, Error);
//            double power = PID * maxSpeed;
//
//            turn(direction, power);
//
//            return 0;
//        }
//        return 1;
//    }
//
//    private void OpenCamDevice(){
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
//                camera.startStreaming(Cam.resolution_x, Cam.resolution_y, OpenCvCameraRotation.UPRIGHT);
//                // Usually this is where you'll want to start streaming from the camera (see section 4)
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//    }
//
//    private void turn(double direction, double power){//       directie: -1 = stanga, 1 = dreapta
//        //luati din mecanum doar turnul
//    }
//
//}
