//package org.firstinspires.ftc.teamcode.experiments;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.hardware.SmartMotor;
//import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
//
//
//@TeleOp
//@Config
//public class AdvancedSliderController extends LinearOpMode {
//
////    public static PIDCoefficients coeffs;
//    public static double kp = 0;
//    public static double kd = 0;
//    public static double ki = 0;
//
//    ElapsedTime elapsedTime;
//    SmartMotorEx slider;
//
//    public static double maxVel = 25;
//    public static double maxAccel = 40;
//    public static double maxJerk = 100;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        slider = new SmartMotorEx(hardwareMap, "slider", SmartMotor.NeveRest.RPM_1780, SmartMotor.MotorDirection.REVERSE);
//        slider.setRunMode(SmartMotor.RunMode.RawPower);
//
//
//        PIDFController controller = new PIDFController(new PIDCoefficients(kp, kd, ki));
//
//        MotionState initState = new MotionState(0, 0, 0);
//
//        MotionState upState = new MotionState(60, 0, 0);
//
//        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
//                initState,
//                upState,
//                maxVel,
//                maxAccel,
//                maxJerk
//        );
//
//        waitForStart();
//        elapsedTime.startTime();
//
//
//        while (opModeIsActive()){
//            MotionState state = profile.get(elapsedTime.time());
//
//            controller.setTargetPosition(state.getX());
//            controller.setTargetVelocity(state.getV());
//            controller.setTargetAcceleration(state.getA());
//
//            double correction = controller.update(slider.getCurrentPosition());
//
//            slider.setPower(correction);
//
//            telemetry.addData("correction", correction);
//            telemetry.addData("power", slider.get());
//            telemetry.addData("target", upState.getX());
//            telemetry.addData("position", slider.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//}
