//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
//import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;
//
//@TeleOp
//public class Slider extends LinearOpMode {
//
//    SmartMotorEx slider;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        slider = new SmartMotorEx(hardwareMap, "left_Slider", SmartMotor.GoBILDA.RPM_223);
//
//        SliderManager sliderManager = new SliderManager( this, slider, gamepad1);
//
//        waitForStart();
//        sliderThread.start();
//
//        while (opModeIsActive()){
//            TelemetryManager();
//        }
//        sliderThread.interrupt();
//    }
//
//
//    public void TelemetryManager(){
//
//        telemetry.addData("Position", slider.getCurrentPosition());
////        telemetry.addData("Target", slider.getTarge());
//        telemetry.addData("Power", slider.get());
//
//        telemetry.update();
//    }
//}
