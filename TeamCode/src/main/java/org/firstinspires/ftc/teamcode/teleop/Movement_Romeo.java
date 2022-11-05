package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotor;
import org.firstinspires.ftc.teamcode.hardware.customHardware.SmartMotorEx;


@TeleOp(group = "advanced")
public class Movement_Romeo extends LinearOpMode {

    String movementType = "center";

//    OpenCvCamera webcam;

    SmartMotorEx frontLeft, backLeft, backRight, frontRight;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Hardware_Map hardware = new Hardware_Map(hardwareMap);

//        SampleMecanumDriveCancelable myLocalizer = new SampleMecanumDriveCancelable(hardwareMap);


        frontLeft = new SmartMotorEx(hardwareMap, "fl", SmartMotor.GoBILDA.RPM_312);
        backLeft = new SmartMotorEx(hardwareMap, "bl", SmartMotor.GoBILDA.RPM_312);
        backRight = new SmartMotorEx(hardwareMap, "br", SmartMotor.GoBILDA.RPM_312);
        frontRight = new SmartMotorEx(hardwareMap, "fr", SmartMotor.GoBILDA.RPM_312);

        frontLeft.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }


//        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        frontLeft.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

//        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
//            myLocalizer.update();
//
//            Pose2d myPose = myLocalizer.getPoseEstimate();

//            telemetry.addData("x", myPose.getX());
//            telemetry.addData("y", myPose.getY());
//            telemetry.addData("heading", myPose.getHeading());
            telemetry.addData("Movement Mode", movementType);
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if(!gamepad1.left_bumper){
                x /= 2;
                y /= 2;
            }
            if(!gamepad1.right_bumper){
                turn /= 2;
            }

            if(gamepad1.dpad_left) movementType = "center";
            else if(gamepad1.dpad_right) movementType = "pivot";

            Movement(y, x, turn, movementType);
            TelemetryManager();

        }

//        webcam.stopStreaming();
    }

    public void Movement(double Forward, double Strafe, double Turn, String movementType) {

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        if(movementType.equals("center")){
            final double v1 = (r * Math.cos(robotAngle)) + Turn;
            final double v2 = (r * Math.sin(robotAngle)) - Turn;
            final double v3 = (r * Math.sin(robotAngle)) + Turn;
            final double v4 = (r * Math.cos(robotAngle)) - Turn;

            frontLeft.motorEx.setPower(v1);
            frontRight.motorEx.setPower(v2);
            backLeft.motorEx.setPower(v3);
            backRight.motorEx.setPower(v4);
        }
        else if(movementType.equals("pivot")){
            final double v1 = (r * Math.cos(robotAngle)) + Turn;
            final double v2 = (r * Math.sin(robotAngle)) - Turn;
            final double v3 = (r * Math.sin(robotAngle)) + Turn/4;
            final double v4 = (r * Math.cos(robotAngle)) - Turn/4;

            frontLeft.motorEx.setPower(v1);
            frontRight.motorEx.setPower(v2);
            backLeft.motorEx.setPower(v3);
            backRight.motorEx.setPower(v4);

        }
    }

    public void TelemetryManager(){
        telemetry.addData("front left", frontLeft.motorEx.getCurrentPosition());
        telemetry.addData("back left", backLeft.motorEx.getCurrentPosition());
        telemetry.addData("back right", backRight.motorEx.getCurrentPosition());
        telemetry.addData("front right", frontRight.motorEx.getCurrentPosition());
        telemetry.addData("direction", frontLeft.motorEx.getDirection().toString() +' '+ backLeft.motorEx.getDirection().toString()+' '+ backRight.motorEx.getDirection().toString()+' '+frontRight.motorEx.getDirection());

        telemetry.update();
    }
}