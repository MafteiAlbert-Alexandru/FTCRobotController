package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;


@Config
@TeleOp(name="1_Movement")
public class Movement extends LinearOpMode {

    public static double ANGLE = 90; // deg

    String movementType = "center";

    SmartMotorEx frontLeft, backLeft, backRight, frontRight;

    Servo servoArm, joint, hook;
    DcMotor leftIntake, rightIntake;

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

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new SmartMotorEx(hardwareMap, "fl", SmartMotor.GoBILDA.RPM_312);
        backLeft = new SmartMotorEx(hardwareMap, "bl", SmartMotor.GoBILDA.RPM_312);
        backRight = new SmartMotorEx(hardwareMap, "br", SmartMotor.GoBILDA.RPM_312);
        frontRight = new SmartMotorEx(hardwareMap, "fr", SmartMotor.GoBILDA.RPM_312);

        frontLeft.motorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.motorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.motorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.motorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        servoArm = hardwareMap.get(Servo.class, "arm");

        joint = hardwareMap.get(Servo.class, "joint");

        hook = hardwareMap.get(Servo.class, "hook");

        servoArm.setDirection(Servo.Direction.FORWARD);



        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        while (opModeIsActive()) {

            if(usingController){
                InputManager();
            }
            else{
                setServoPos(servoPosArm);
                setIntakePower(intakePow);
            }
            // TelemetryManager();

            if (gamepad1.dpad_left) movementType = "center";
            else if (gamepad1.dpad_right) movementType = "pivot";

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (!gamepad1.left_bumper) {
                x /= 3;
                y /= 3;
            }
            if (!gamepad1.right_bumper) {
                turn /= 3;
            }

            Mecanum(y, x, turn, movementType, ANGLE);

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
    }


    public void Mecanum(double Forward, double Strafe, double Turn, String movementType, double angle) {

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        if(movementType.equals("center")){
            final double v1 = (r * Math.cos(robotAngle)) + Turn;
            final double v2 = (r * Math.sin(robotAngle)) - Turn;
            final double v3 = (r * Math.sin(robotAngle)) + Turn;
            final double v4 = (r * Math.cos(robotAngle)) - Turn;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);
        }
        else if(movementType.equals("pivot")){
            final double v1 = (r * Math.cos(robotAngle)) + Turn/angle;
            final double v2 = (r * Math.sin(robotAngle)) - Turn/angle;
            final double v3 = (r * Math.sin(robotAngle)) + Turn;
            final double v4 = (r * Math.cos(robotAngle)) - Turn;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

        }
    }

    public void TelemetryManager() {
        telemetry.addLine("front left" + frontLeft.motorEx.getCurrentPosition());
        telemetry.addLine("back left" + backLeft.motorEx.getCurrentPosition());
        telemetry.addLine("back right" + backRight.motorEx.getCurrentPosition());
        telemetry.addLine("front right" +  frontRight.motorEx.getCurrentPosition());
        telemetry.addLine("Left Intake Power" + leftIntake.getPower()*100);
        telemetry.addLine("Right Intake Power" + rightIntake.getPower()*100);
        telemetry.addLine("Intake Power" + (leftIntake.getPower()+rightIntake.getPower())/2*100);
        telemetry.addLine("Arm pos" + servoArm.getPosition());
        telemetry.update();
        }
}