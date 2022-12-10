//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
//import org.firstinspires.ftc.teamcode.hardware.customHardware.ServoMirror;
//
//@TeleOp
//@Config
//public class CrocodiluTeleOp extends LinearOpMode {
//
//    Servo jointLeft, jointRight;
//    Servo claw;
//    ServoMirror joint;
//    ColorRangeSensor sensorColor;
//    public double initJoint = 0;
//    public double initClaw = 0;
//    public double openClaw = 0;
//    public double closeClaw = 0;
//    public double jointPos = 0;
//    public boolean usingGamepad = false;
//    boolean coneInClaw = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        jointLeft = hardwareMap.get(Servo.class, "jointL");
//        jointRight = hardwareMap.get(Servo.class, "jointR");
//        claw = hardwareMap.get(Servo.class, "claw");
//
//        sensorColor = hardwareMap.get(ColorRangeSensor.class, "sensorColor");
//
//        joint = new ServoMirror(jointLeft, jointRight);
//
//        waitForStart();
//
//        joint.setPosition(initJoint);
//        claw.setPosition(initClaw);
//
//        while (opModeIsActive()){
//
//            if(usingGamepad) GamepadControl();
//            else AutoControl();
//
//        }/
//    }
//
//    void GamepadControl(){
//        if(gamepad1.left_bumper) joint.addPosition(0.0001);
//        else if(gamepad1.right_bumper) joint.addPosition(-0.0001);
//    }
//
//    void AutoControl(){
//
//    }
//
//    boolean ConeFound(){
//        if(/* conditia care se verifica de la senzor */){
//            coneInClaw = true;
//            return true;
//        }
//        else return false;
//    }
//}
