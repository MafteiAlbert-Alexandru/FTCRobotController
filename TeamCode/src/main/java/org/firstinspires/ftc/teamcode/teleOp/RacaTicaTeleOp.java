//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.hardware.SmartMotorEx;
//
//@TeleOp
//@Config
//public class RacaTicaTeleOp extends LinearOpMode {
//
//    DcMotor fl, bl, br, fr;
//    SmartMotorEx slider;
//    double x, y, turn;
//
//    Servo claw;
//
//    public static double PosCoefficient = 0.05;
//    public static double PosTolerance = 8;
//    public static double sliderPower = 0.1;
//    public static int HighPos = 4300;
//    public static int MidPos = 2250;
//    public static int LowPos = 1000;
//    public static int GroundPos = 10;
//    public static double power = 0.05; //Thw Power of the slides
//    public static double DPP = 1; //Distance per Pulse
//
//    public static double closedPos = 0;
//    public static double openPos = 0;
//
//    boolean isPressed = false;
//    boolean closed = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        fl.setDirection(DcMotorSimple.Direction.FORWARD);
//        bl.setDirection(DcMotorSimple.Direction.FORWARD);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//
////        slider = new SmartMotorEx(hardwareMap, "slider", SmartMotor.GoBILDA.RPM_223);
//
////        claw = hardwareMap.get(Servo.class, "claw");
//
////        Thread sliderThread = new Thread( () -> {
////           SliderManager(true);
////        });
//
//        waitForStart();
//
////        sliderThread.start();
//        while (opModeIsActive()){
//            InputManager();
//            Movement();
////            TelemetryManager();
//        }
//    }
//
//    void ClawController(){
//        if(closed) claw.setPosition(closedPos);
//        else claw.setPosition(openPos);
//    }
//
//    void Movement() {
//
//        double Strafe = x;
//        double Forward = y;
//        double Turn = turn;
//
//        double r = Math.hypot(Strafe, Forward);
//
//        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;
//
//        final double v1 = (r * Math.cos(robotAngle)) + Turn;
//        final double v2 = (r * Math.sin(robotAngle)) - Turn;
//        final double v3 = (r * Math.sin(robotAngle)) + Turn;
//        final double v4 = (r * Math.cos(robotAngle)) - Turn;
//
//        fl.setPower(v1);
//        fr.setPower(v2);
//        bl.setPower(v3);
//        br.setPower(v4);
//
//    }
//
//    void InputManager() {
////
////        if(gamepad1.a && !isPressed){
////            isPressed = true;
////            closed =! closed;
////        }
////        else if(!gamepad1.a) isPressed = false;
////
////        ClawController();
//
//        x = gamepad1.left_stick_x;
//        y = -gamepad1.left_stick_y;
//        turn = gamepad1.right_stick_x;
//
//        if(!gamepad1.left_bumper)
//        {
//            x = x/2;
//            y = y/2;
//        }
//
//        if(!gamepad1.right_bumper)
//        {
//            turn = turn/2;
//        }
//    }
//
//    void TelemetryManager(){
//        telemetry.addData("Position Left", slider.getCurrentPosition());
//        telemetry.update();
//    }
//
//    void SliderManager(boolean isInverted){
//
//        slider.setInverted(isInverted);
//
//        slider.setRunMode(SmartMotorEx.RunMode.PositionControl);
//        slider.setPositionCoefficient(PosCoefficient);
//        slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
//        slider.setPositionTolerance(PosTolerance);
//        slider.resetEncoder();
//
//        while(opModeIsActive()){
//            if(gamepad1.left_bumper){
//                slider.setTargetPosition(HighPos);
//                if(slider.getCurrentPosition()<HighPos){
//                    slider.set(1);
//                }
//                else slider.stopMotor();
//            }
//            else if(gamepad1.right_bumper){
//                slider.setTargetPosition(GroundPos);
//                if(slider.getCurrentPosition()>GroundPos){
//                    slider.set(1);
//                }
//                else
//                    slider.stopMotor();
//            }
//            else slider.stopMotor();
//
//            if(gamepad1.dpad_down) {
//                LowPosSlider(LowPos);
//            }
//            else if(gamepad1.dpad_left) {
//                MidPosSlider(MidPos);
//            }
//            else if (gamepad1.dpad_up) {
//                HighPosSlider(HighPos);
//            }
//            else if (gamepad1.dpad_right){
//                GroundPosSlider(GroundPos);
//            }
//        }
//    }
//
//    void HighPosSlider ( int pos){
//
//        SetPosSlider(pos);
//
//        String direction;
//
//        if (slider.getCurrentPosition() > pos) direction = "down";
//        else if (slider.getCurrentPosition() < pos) direction = "up";
//        else direction = "equal";
//
//        if (direction.equals("up")) {
//            while (slider.getCurrentPosition() <= pos && opModeIsActive()) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.stopMotor();
//                if (gamepad1.touchpad) break;
//            }
//        } else if (direction.equals("down")) {
//            while (slider.getCurrentPosition() >= pos && opModeIsActive()) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.stopMotor();
//                if (gamepad1.touchpad) break;
//            }
//        } else slider.stopMotor();
//    }
//
//    void MidPosSlider ( int pos){
//        SetPosSlider(pos);
//
//        String direction;
//
//        if (slider.getCurrentPosition() > pos) direction = "down";
//        else if (slider.getCurrentPosition() < pos) direction = "up";
//        else direction = "equal";
//
//        if (direction.equals("up")) {
//            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else if (direction.equals("down")) {
//            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else slider.stopMotor();
//    }
//
//    void LowPosSlider ( int pos){
//        SetPosSlider(pos);
//
//        String direction;
//
//        if (slider.getCurrentPosition() > pos) direction = "down";
//        else if (slider.getCurrentPosition() < pos) direction = "up";
//        else direction = "equal";
//
//        if (direction.equals("up")) {
//            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else if (direction.equals("down")) {
//            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else slider.stopMotor();
//    }
//
//    void GroundPosSlider(int pos){
//        SetPosSlider(pos);
//
//        String direction;
//
//        if (slider.getCurrentPosition() > pos) direction = "down";
//        else if (slider.getCurrentPosition() < pos) direction = "up";
//        else direction = "equal";
//
//        if (direction.equals("up")) {
//            while (opModeIsActive()) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else if (direction.equals("down")) {
//            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
//                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
//                else slider.set(power);
//                if (gamepad1.touchpad) break;
//            }
//        } else slider.stopMotor();
//    }
//
//    void SetPosSlider(int pos) {
//        slider.setTargetPosition(pos);
//    }
//}
