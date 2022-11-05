package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.TransferSubsystem;


@TeleOp
@Config
public class IntakeSystem_V2 extends LinearOpMode {

    DcMotorEx leftIntake, rightIntake;
    GamepadEx drivePad;
    Servo arm, hook, joint;

    DcMotorEx frontLeft, backLeft, backRight, frontRight;

    SmartMotorEx Left_Slider, Right_Slider;

    private boolean liftCase, jointCase, hookCase;
    public double downLiftPos = 0.41;
    public double upLiftPos = 0;
    public double closeJointPos = 1;
    public double openJointPos = 0.57;
    public double closeHookPos = 0.64;
    public double openHookPos = 0.58;

    public static double PosCoefficient = 0.05;
    public static double PosTolerance = 8;
    public static double sliderPower = 0.1;
    public static int HighPos = 3500;
    public static int MidPos = 2250;
    public static int LowPos = 1000;
    public static int GroundPos = 10;
    public int leftPos, rightPos;

    public static double power = 0.05; //Thw Power of the slides
    public static double DPP = 1; //Distance per Pulse

    @Override
    public void runOpMode() throws InterruptedException {

        drivePad = new GamepadEx(gamepad1);

        arm = hardwareMap.get(Servo.class, "arm");
        joint = hardwareMap.get(Servo.class, "joint");
        hook = hardwareMap.get(Servo.class, "hook");

        //region movement motors
        boolean IntakeBool = false;
        boolean liftBool = false;
        boolean jointBool = false;
        boolean hookBool = false;

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");


        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        frontRight.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");

        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        //endregion

        //region transfer system
        //endregion

        //region slider motors
        Left_Slider = new SmartMotorEx(hardwareMap, "Left_Slider", SmartMotorEx.GoBILDA.RPM_223);
        Right_Slider = new SmartMotorEx(hardwareMap, "Right_Slider", SmartMotorEx.GoBILDA.RPM_223);

        Thread left_sliderThread = new Thread( () -> SliderManager(Left_Slider, true));
        Thread right_sliderThread = new Thread( () -> SliderManager(Right_Slider, false));
        //endregion

//        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(leftIntake, rightIntake);
//        TransferSubsystem transferSubsystem = new TransferSubsystem(arm, hook, joint);

        ///Intake
//        drivePad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new RunCommand(intakeSubsystem::IntakePower));
//
//        ///Transfer
//        drivePad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new RunCommand(transferSubsystem::TransferLift));
//
//        drivePad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new RunCommand(transferSubsystem::TransferHook));
//
//        drivePad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new RunCommand(transferSubsystem::TransferJoint));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        left_sliderThread.start();
        right_sliderThread.start();



        while (opModeIsActive()){

            leftIntake.setPower(gamepad1.left_trigger);
            rightIntake.setPower(gamepad1.left_trigger);

//            if(gamepad1.x && IntakeBool){
//                IntakeBool = false;
//
//            }
//            else if(!gamepad1.x) IntakeBool = true;

            if(gamepad1.y && liftBool){
                liftBool = false;
                TransferLift();
            }
            else if(!gamepad1.y) liftBool = true;

            if(gamepad1.b){
                TransferJoint(false);
            }

            if(gamepad1.touchpad) TransferJoint(true);

            if(gamepad1.a && hookBool){
                hookBool = false;
                TransferHook();
            }
            else if(!gamepad1.a) hookBool = true;


            leftPos = Left_Slider.getCurrentPosition();
            rightPos = Right_Slider.getCurrentPosition();

            Left_Slider.setDistancePerPulse(DPP);
            Right_Slider.setDistancePerPulse(DPP);

            drivePad.readButtons();
            MovementMecanum(drivePad.getLeftY(), drivePad.getLeftX(), drivePad.getRightX());
            TelemetryManager();
        }
    }



    void TelemetryManager(){
        telemetry.addLine("Position Left " + Left_Slider.getCurrentPosition());
        telemetry.addLine("Position Right " + Right_Slider.getCurrentPosition());
//        telemetry.addLine("front left " + frontLeft.motorEx.getCurrentPosition());
//        telemetry.addLine("back left " + backLeft.motorEx.getCurrentPosition());
//        telemetry.addLine("back right " + backRight.motorEx.getCurrentPosition());
//        telemetry.addLine("front right " +  frontRight.motorEx.getCurrentPosition());
        telemetry.addLine("Left Intake Power " + leftIntake.getPower()*100);
        telemetry.addLine("Right Intake Power " + rightIntake.getPower()*100);
        telemetry.addLine("Intake Power " + (leftIntake.getPower()+rightIntake.getPower())/2*100);
        telemetry.addLine("Arm pos" + arm.getPosition());
        telemetry.update();
    }

    void MovementMecanum(double Forward, double Strafe, double Turn){

        if(!gamepad1.left_stick_button){
            Forward /= 3;
            Strafe  /= 3;
        }
        if(!gamepad1.right_stick_button){
            Turn /= 3;
        }

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + Turn;
        final double v2 = (r * Math.sin(robotAngle)) - Turn;
        final double v3 = (r * Math.sin(robotAngle)) + Turn;
        final double v4 = (r * Math.cos(robotAngle)) - Turn;

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);

    }

    public void TransferLift(){
        liftCase =! liftCase;

        if(liftCase) {
            arm.setPosition(upLiftPos);
        }
        else arm.setPosition(downLiftPos);
    }

    public void TransferJoint(boolean bool){
        //jointCase =! jointCase;

        if(bool) {
            joint.setPosition(openJointPos);
        }
        else joint.setPosition(closeJointPos);
    }

    public void TransferHook(){
        hookCase =! hookCase;

        if(hookCase) hook.setPosition(closeHookPos);
        else hook.setPosition(openHookPos);
    }

    void SliderManager(SmartMotorEx slider, boolean isInverted){

        slider.setInverted(isInverted);

        slider.setRunMode(SmartMotorEx.RunMode.PositionControl);
        slider.setPositionCoefficient(PosCoefficient);
        slider.setZeroPowerBehavior(SmartMotorEx.ZeroPowerBehavior.BRAKE);
        slider.setPositionTolerance(PosTolerance);
        slider.resetEncoder();

        while(opModeIsActive()){
            if(gamepad1.left_bumper){
                slider.setTargetPosition(HighPos);
                if(slider.getCurrentPosition()<HighPos){
                    slider.set(1);
                }
                else slider.stopMotor();
            }
            else if(gamepad1.right_bumper){
                slider.setTargetPosition(GroundPos);
                if(slider.getCurrentPosition()>GroundPos){
                    slider.set(1);
                }
                else
                    slider.stopMotor();
            }
            else slider.stopMotor();

            if(gamepad1.dpad_down) {
                LowPosSlider(LowPos, slider);
            }
            else if(gamepad1.dpad_left) {
                MidPosSlider(MidPos, slider);
            }
            else if (gamepad1.dpad_up) {
                HighPosSlider(HighPos, slider);
            }
            else if (gamepad1.dpad_right){
                GroundPosSlider(GroundPos, slider);
            }
        }
    }

    void HighPosSlider ( int pos, SmartMotorEx slider){

        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (slider.getCurrentPosition() <= pos && opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.stopMotor();
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (slider.getCurrentPosition() >= pos && opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.stopMotor();
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void MidPosSlider ( int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void LowPosSlider ( int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive() && slider.getCurrentPosition() <= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void GroundPosSlider(int pos, SmartMotorEx slider){
        SetPosSlider(pos, slider);

        String direction;

        if (slider.getCurrentPosition() > pos) direction = "down";
        else if (slider.getCurrentPosition() < pos) direction = "up";
        else direction = "equal";

        if (direction.equals("up")) {
            while (opModeIsActive()) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else if (direction.equals("down")) {
            while (opModeIsActive() && slider.getCurrentPosition() >= pos) {
                if (slider.getCurrentPosition() >= 1000) slider.set(sliderPower);
                else slider.set(power);
                if (gamepad1.touchpad) break;
            }
        } else slider.stopMotor();
    }

    void SetPosSlider(int pos, @NonNull SmartMotorEx motor) {
        motor.setTargetPosition(pos);
    }
}
