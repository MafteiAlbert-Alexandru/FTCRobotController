package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
public class IntakeTest_TeleOp extends LinearOpMode {

    //public static boolean emergencyStop = false; //STOP
    public static boolean usingController = true; //Controller Mode
    public static double servoPosArm = 0.55; //Servo Left Position
//    private static double servoPosR = 0.55; //Servo Right Position
    public static double intakePow = 0; //Intake Power
    public static double servoIntakePos = 0.46;
//    private static double IntakePosR = 0.55;
    public static double MidPos = 0.485;
    public static double servoDropPosL = 0;
//    private static double DropPosR = 0.16;
    public static double IntakePower = 0.5;

    Servo servoArm;
    DcMotor leftIntake, rightIntake;

    GamepadEx driverController = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        servoArm = hardwareMap.get(Servo.class, "arm");
//        rightArm = hardwareMap.get(Servo.class, "rightArm");

        servoArm.setDirection(Servo.Direction.FORWARD);
//        rightArm.setDirection(Servo.Direction.REVERSE);

//        servoArm = new SmartServo(hardwareMap, "servoArm", -150, 150, AngleUnit.DEGREES);

//        servoArm.setInverted(false);
//        rightArm.setInverted(true);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);


//        Thread servoArmDropPos = new Thread( () -> {

//            SmartServo threadServo =
//            servoArm;

//            double servoSpeed = -0.5;
//
//            while(servoArm.getAngle()> servoArm.getAngleFromPos(DropPos)){
//
//                if(servoArm.getAngle() + servoSpeed < servoArm.getAngleFromPos(DropPos)){
//                    servoArm.rotateByAngle(-0.5);
//                    servoSpeed -= 0.02;
//                }
//                else {
//                    servoArm.turnToAngle(servoArm.getAngleFromPos(DropPos));
//                }
//            }

//        });

//
//        GamepadButton dropIntakeButton = new GamepadButton(
//                driverController, GamepadKeys.Button.X
//        );
//        GamepadButton raiseIntakeButton = new GamepadButton(
//                driverController, GamepadKeys.Button.B
//        );
//
//        GamepadButton midIntake = new GamepadButton(
//                driverController, GamepadKeys.Button.Y
//        );
//
//        ToggleButtonReader toggleIntakePower = new ToggleButtonReader(
//                driverController, GamepadKeys.Button.A
//        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            if(usingController){
                InputManager();
            }
            else{
                setServoPos(servoPosArm);
                setIntakePower(intakePow);
            }
            TelemetryManager();
        }
    }

    void setServoPos(double posL){
        servoArm.setPosition(posL);
//        rightArm.setPosition(posR);
    }

    void setIntakePower(double pow){
        leftIntake.setPower(pow);
        rightIntake.setPower(pow);
    }


    void InputManager(){
        if(gamepad1.dpad_left) setIntakePower(0);
        else if(gamepad1.dpad_right) setIntakePower(IntakePower);


        if(gamepad1.square) setServoPos(servoIntakePos);
        else if(gamepad1.circle) setServoPos(servoDropPosL);
        if(gamepad1.y) setServoPos(MidPos);

   //     if(gamepad1.touchpad) emergencyStop = true;

        driverController.readButtons();
    }

    void TelemetryManager(){
        telemetry.addLine("Left Intake Power" + leftIntake.getPower()*100);
        telemetry.addLine("Right Intake Power" + rightIntake.getPower()*100);
        telemetry.addLine("Intake Power" + (leftIntake.getPower()+rightIntake.getPower())/2*100);
        telemetry.addLine("Arm pos" + servoArm.getPosition());
        telemetry.update();
    }
}
